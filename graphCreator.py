import networkx as nx
import numpy as np
import copy
import itertools
import math
import matplotlib.pyplot as plt #do wizualizaccji i rysowania grafu
import json 
from shapely.geometry import LineString, Point
from shapely.ops import unary_union
from dispatcher import Task

blockWeight = 9999999
corridorWidth = 0.3 #1
robotLength= 0.2 #0.6

wayType = {
    "twoWay": 1,
    "narrowTwoWay": 2,
    "oneWay": 3
}
baseNodeSectionType = {
    "dockWaitUndock": 1, #rozbicie na dock, wait, undock, end (przesunąć współrzędne węzłów tylko do wyświetlania i diagnostyki, docelowo można pominąć)
    "waitPOI": 2, #rozbicie na wait, end (przesunąć współrzędne węzłów tylko do wyświetlania i diagnostyki, docelowo można pominąć)
    "noChanges": 3, #brak zmian, przepisać
    "normal": 4, #pomijane przy rozbijaniu węzłów, ale uwzględniane przy kształcie korytarza
    "intersection": 5 #rozbicie na in, out - liczba zależy od krawędzi związanych z węzłem - konieczność wygenerowania prawidłowych współrzędnych przesunięcia
}
baseNodeType ={
    "charger": {"id": 1, "nodeSection": baseNodeSectionType["dockWaitUndock"]}, #charger                               
    "load": {"id": 2, "nodeSection": baseNodeSectionType["waitPOI"]}, #loading POI                                     
    "unload": {"id": 3, "nodeSection": baseNodeSectionType["waitPOI"]}, #unloading POI                         
    "load-unload": {"id": 4, "nodeSection": baseNodeSectionType["waitPOI"]}, #loading-unloading POI                      
    "load-dock": {"id": 5, "nodeSection": baseNodeSectionType["dockWaitUndock"]}, #loading-docking POI                    
    "unload-dock": {"id": 6, "nodeSection": baseNodeSectionType["dockWaitUndock"]}, #unloading-docking POI               
    "load-unload-dock": {"id": 7, "nodeSection": baseNodeSectionType["dockWaitUndock"]}, #loading-unloading-docking POI 
    "waiting": {"id": 8, "nodeSection": baseNodeSectionType["noChanges"]}, #waiting POI                                  
    "departure": {"id": 9, "nodeSection": baseNodeSectionType["noChanges"]}, #departue POI                               
    "waiting-departure": {"id": 10, "nodeSection": baseNodeSectionType["intersection"]}, #waiting-departure-POI             
    "parking": {"id": 11, "nodeSection": baseNodeSectionType["noChanges"]}, #parking POI                                 
    "queue": {"id": 12, "nodeSection": baseNodeSectionType["noChanges"]}, #queue POI                                
    "normal": {"id": 13, "nodeSection": baseNodeSectionType["normal"]}, #normal POI                              
    "intersection": {"id": 14, "nodeSection": baseNodeSectionType["intersection"]}, #intersection POI               
    }
newNodeType = {
    #type - służy do przyszłego złączenia krawędzi grafu w odpowiedniej kolejności 
            #znaczenie typow wezlow
    "dock": 1, #rozpoczecie dokowania
    "wait": 2, #rozpoczecie wlaciwej operacji na stanowisku
    "undock": 3,#rozpoczecie procedury oddokowania
    "end": 4, #zakonczenie zadania, mozliwosc przekierowania robota do innego zadania (wolny) lub kolejnego zachowania
    "noChanges": 5, #wezel nie ulega zmianom wzgledem tego od ktorego jest tworzony
    "intersection_in": 6, #wjazd na skrzyzowanie
    "intersection_out": 7 #zjazd ze skrzyzowania
}
nodeColor = { #do celow wizualizacji i podgladu
    "dockWaitUndock": "yellow", #L
    "dock": "#00fc00", #jasna zielen
    "wait": "red", 
    "undock": "#00a800", #ciemna zielen
    "end": "#8c0000", #ciemny czerwony
    "noChanges": "orange", #S
    "intersection_base": "blue", #D
    "in": "black", #P
    "out": "magenta", #WP 
    "skipped" : "#72fdfe" #blękit - wezly ktore powinny byc pominiete, ale chyba beda rysowane
}

class DataConverter():
    def __init__(self, sourceNodes, sourceEdges):
        self.convertDataRun(sourceNodes, sourceEdges)
    
    def convertDataRun(self, sourceNodes, sourceEdges):
        #odczyt danych, wywołanie odpowiednich funkcji i utworzenie właściwego grafu
        #TODO odkomentowac 2 ponizsze linie jestli na werjsciu maja byc dane z grafu odczytanego z bazy odpowiednio dla wezlow i krawedzi
        #self.sourceNodes = self.convertDatabaseNodes(sourceNodes)
        #self.sourceEdges = self.convertDatabaseEdges(sourceEdges)
        #na potrzeby testow
        #TODO zakomentowac 2 ponizsze linie jest wkorzystane maja byc jako dane wejsciowe dane z bazy danych
        self.sourceNodes = sourceNodes
        self.sourceEdges = sourceEdges       
        edges = self.createEdges(copy.deepcopy(sourceEdges))
        self.reducedEdges = self.combinedNormalNodes(edges)
        self.validatePoiConnectionNodes()
        self.validateParkingConnectionNodes()
        self.validateQueueConnectionNodes()
        self.validateWaitingConnectionNodes()
        self.validateDepartureConnectionNodes()
        self.validateWaitDepConnectionNodes()
        print("dane prawidłowe") 
        
    def convertDatabaseNodes(self, sourceNodes):
        """
        Konwersja danych z bazy danych do obslugiwanego formatu przez klasy.
        sourceNodes
        {id: {"name": string, 
              "pos": (x,y), 
              "type": GraphDataConverter.baseNodeType["type"], 
              "poiID": int}, ...}
        """
        databaseNodes=json.loads(sourceNodes)
        convertedNodes = {}
        for node in databaseNodes:
            convertedNodes[node["id"]] = {"name": node["name"], "pos": (node["posX"],node["posY"]), 
                                    "type": node["type"], "poiID": node["poiID"]} 
        return convertedNodes  
            
    def convertDatabaseEdges(self, sourceEdges):
        """
        Konwersja danych z bazy danych do obslugiwanego formatu przez klasy.
        sourceEdges
        {id: {"startNode": string, 
              "endNode"
              "type":  GraphDataConverter.wayType["type"], 
              }, ...}
        """
        databaseEdges=json.loads(sourceEdges)
        convertedEdges = {}
        for edge in databaseEdges:
            wayType = 0
            if edge["biDirected"] and edge["narrow"]:
                wayType = wayType["twoWay"]
            elif edge["biDirected"] and edge["narrow"] == False:
                wayType = wayType["narrowTwoWay"]
            else:
                wayType = wayType["oneWay"]
            convertedEdges[edge["id"]] = {
                "startNode": edge["vertexB"]["id"],
                "endNode": edge["vertexA"]["id"],
                "type": wayType,
                "isActive": edge["isActive"]
            }
        return convertedEdges
    
    def createEdges(self, sourceEdges):
        i = 1
        extendedEdges = {}
        for k in sourceEdges:
            edge = sourceEdges[k]
            if edge["type"] == wayType["twoWay"] or \
                edge["type"] == wayType["narrowTwoWay"]:
                    
                a = edge["startNode"]
                b = edge["endNode"]
                extendedEdges[i] = edge.copy()
                extendedEdges[i]["startNode"] = a
                extendedEdges[i]["endNode"] = b
                extendedEdges[i]["edgeSource"] = k
                i = i + 1
                extendedEdges[i] = edge.copy()
                extendedEdges[i]["startNode"] = b
                extendedEdges[i]["endNode"] = a
                extendedEdges[i]["edgeSource"] = k
                i = i + 1
                pass
            else: 
                extendedEdges[i] = edge.copy()
                extendedEdges[i]["edgeSource"] = k
                i = i + 1
        return extendedEdges
   
    def combinedNormalNodes(self,edges):
        normalNodesId = {i for i in self.sourceNodes if self.sourceNodes[i]["type"] == baseNodeType["normal"]}
        combinedPath = {edgeId:{"edgeSourceId": [edges[edgeId]["edgeSource"]],
                                "connectedNodes": [edges[edgeId]["startNode"], edges[edgeId]["endNode"]]} 
                        for edgeId in edges 
                        if edges[edgeId]["startNode"] not in normalNodesId
                           and edges[edgeId]["endNode"] in normalNodesId}
        edgesNormalNodes = [i for i in edges if edges[i]["startNode"] in normalNodesId]
        previosEdgeLen = len(edgesNormalNodes)
        while True:
            for i in edgesNormalNodes:
                edge = edges[i]
                for j in combinedPath:
                    node = combinedPath[j]
                    if node["connectedNodes"][-1] == edge["startNode"] and \
                        edge["edgeSource"] not in node["edgeSourceId"]:
                        combinedPath[j]["connectedNodes"].append(edge["endNode"])
                        combinedPath[j]["edgeSourceId"].append(edge["edgeSource"])
                        edgesNormalNodes.remove(i)
                        break
                    
            if(previosEdgeLen == len(edgesNormalNodes)):
                assert len(edgesNormalNodes) == 0, "Normal nodes error. Path contains normal nodes should start and end from different type of node."              
                break
            else:
                previosEdgeLen = len(edgesNormalNodes)
        self._validateDirectionCombinedPath(combinedPath)
        return self._combinedNormalNodesEdges(edges,combinedPath)
        
    def _validateDirectionCombinedPath(self,connectedNormalNodesPaths):
        combinedPath = copy.deepcopy(connectedNormalNodesPaths)
        for i in combinedPath:
            edgesId = combinedPath[i]["edgeSourceId"]
            previousPathType = self.sourceEdges[edgesId[0]]["type"]
            for j in range(len(edgesId)-1):
                currentPathType = self.sourceEdges[edgesId[j+1]]["type"]
                assert previousPathType == currentPathType, "Different path type connected to normal nodes."
            endNodeId = combinedPath[i]["connectedNodes"][-1]
            endNodeType = self.sourceNodes[endNodeId]["type"]
            assert endNodeType != baseNodeType["normal"],"Path contains normal nodes should be end different type of nodes."
                
    def _combinedNormalNodesEdges(self, edges, combinedPath):
        #utworzenie listy krawędzi z pominięciem wezlow normalnych do dalszych analiz polaczen miedzy
        #typami wezlow
        reducedEdges = {}
        j = 1
        for i in edges:
            edge = edges[i]
            if self.sourceNodes[edge["startNode"]]["type"] != baseNodeType["normal"] \
                and self.sourceNodes[edge["endNode"]]["type"] != baseNodeType["normal"]:
                if j not in reducedEdges:
                    reducedEdges[j] = {"sourceEdges": [], "sourceNodes": [], "wayType": 0}
                reducedEdges[j]["sourceEdges"] = [edge["edgeSource"]]
                reducedEdges[j]["sourceNodes"] = [edge["startNode"],edge["endNode"]]
                reducedEdges[j]["wayType"] = edge["type"]
                j = j + 1
            elif self.sourceNodes[edge["startNode"]]["type"] != baseNodeType["normal"] \
                and self.sourceNodes[edge["endNode"]]["type"] == baseNodeType["normal"]:
                lastNodeId = [pathId for pathId in combinedPath \
                              if combinedPath[pathId]["connectedNodes"][0] == edge["startNode"]\
                                and combinedPath[pathId]["connectedNodes"][1] == edge["endNode"]]
                if j not in reducedEdges:
                    reducedEdges[j] = {"sourceEdges": [], "sourceNodes": [], "wayType": 0}                
                reducedEdges[j]["sourceEdges"] = combinedPath[lastNodeId[0]]["edgeSourceId"]
                reducedEdges[j]["sourceNodes"] = combinedPath[lastNodeId[0]]["connectedNodes"]
                reducedEdges[j]["wayType"] = edge["type"]                
                del combinedPath[lastNodeId[0]]
                j = j + 1
        return reducedEdges
    
    def validatePoiConnectionNodes(self):
        #pobranie wszystkich poi z dokowaniem i bez dokowania
        #sprawdzenie czy kazde POI laczy sie z dwoma innymi wezlami
        # - konfiguracja I waiting Poi, departure poi
        # - walidacja krawedzi obie sa jednokierunkowe, inaczej blad typu krawedzi
        
        # - konfiguracja II waiting-departure poi, poi waiting departure
        # - walidacja krawedzi - powinna byc typu waskiego dwukierunkowa inaczej blad typu krawedzi
        # dla wszystkich innych polaczen blad grafu i polaczen stanowisk
        poiNodesId = [i for i in self.sourceNodes \
                      if self.sourceNodes[i]["type"]["nodeSection"] == baseNodeSectionType["dockWaitUndock"] \
                          or self.sourceNodes[i]["type"]["nodeSection"] == baseNodeSectionType["waitPOI"]]
        for i in poiNodesId:
            inNodes = [self.reducedEdges[j]["sourceNodes"][0] for j in self.reducedEdges \
                       if self.reducedEdges[j]["sourceNodes"][-1] == i] 
            outNodes = [self.reducedEdges[j]["sourceNodes"][-1] for j in self.reducedEdges \
                        if self.reducedEdges[j]["sourceNodes"][0] == i]
            assert len(inNodes) == 1, "Only one waiting/waiting-departure POI should be connected with POI."
            assert len(outNodes) == 1, "Only one departure/waiting-departure POI should be connected with POI."
            
            inNodeType = self.sourceNodes[inNodes[0]]["type"]
            outNodeType = self.sourceNodes[outNodes[0]]["type"]
            
            assert (inNodeType == baseNodeType["waiting"] and outNodeType == baseNodeType["departure"]) \
                    or  (inNodeType == baseNodeType["waiting-departure"] and \
                         outNodeType == baseNodeType["waiting-departure"]), "Connected POI with given nodes not allowed. Available connection: waiting->POI->departure or waiting-departure->POI->waiting-departure."
            
            inEdgeType = [self.reducedEdges[j]["wayType"] for j in self.reducedEdges \
                          if self.reducedEdges[j]["sourceNodes"][0] == inNodes[0] \
                              and self.reducedEdges[j]["sourceNodes"][-1] == i]
            
            outEdgeType = [self.reducedEdges[j]["wayType"] for j in self.reducedEdges \
                          if self.reducedEdges[j]["sourceNodes"][0] == i \
                              and self.reducedEdges[j]["sourceNodes"][-1] == outNodes[0]]
            if inNodeType == baseNodeType["waiting"]:
                assert inEdgeType[0] == wayType["oneWay"] and \
                    outEdgeType[0] == wayType["oneWay"], "Edges should be one way in connection waiting->POI->departure"
            else:
                assert inEdgeType[0] == wayType["narrowTwoWay"] and \
                    outEdgeType[0] == wayType["narrowTwoWay"], "Edges should be narrow two way in connection waiting-departure->POI->waiting-departure"
    
    def validateParkingConnectionNodes(self):
        #pobranie wszystkich miejsc parkingowych
        #sprawdzenie czy parking laczy sie bezposrednio ze skrzyzowaniem
        #krawedz laczaca powinna byc typu waskieg dwukierunkowego
        parkingNodesId = [i for i in self.sourceNodes \
                      if self.sourceNodes[i]["type"] == baseNodeType["parking"]]
        for i in parkingNodesId:
            inNodes = [self.reducedEdges[j]["sourceNodes"][0] for j in self.reducedEdges \
                       if self.reducedEdges[j]["sourceNodes"][-1] == i] 
            outNodes = [self.reducedEdges[j]["sourceNodes"][-1] for j in self.reducedEdges \
                        if self.reducedEdges[j]["sourceNodes"][0] == i]
            assert len(inNodes) == 1, "Only one intersection node should be connected as input with parking."
            assert len(outNodes) == 1, "Only one intersection node should be connected as output with parking."
            inNodeType = self.sourceNodes[inNodes[0]]["type"]
            outNodeType = self.sourceNodes[outNodes[0]]["type"]
            assert (inNodeType == baseNodeType["intersection"] and outNodeType == baseNodeType["intersection"]), "Connected Parking with given nodes not allowed. Available connection: intersection->POI->intersection."
            
            inEdgeType = [self.reducedEdges[j]["wayType"] for j in self.reducedEdges \
                          if self.reducedEdges[j]["sourceNodes"][0] == inNodes[0] \
                              and self.reducedEdges[j]["sourceNodes"][-1] == i]
            
            outEdgeType = [self.reducedEdges[j]["wayType"] for j in self.reducedEdges \
                          if self.reducedEdges[j]["sourceNodes"][0] == i \
                              and self.reducedEdges[j]["sourceNodes"][-1] == outNodes[0]]
            assert inEdgeType[0] == wayType["narrowTwoWay"] and \
                outEdgeType[0] == wayType["narrowTwoWay"], "Edges should be narrow two way in connection intersection->parking->intersection."
    
    def validateQueueConnectionNodes(self):
        #pobranie wszystkich miejsc w ktorych kolejkowane beda roboty do parkowania
        #sprawdzenie czy wezel poprzedzajacy i nastepny lacza sie ze skrzyzowaniem
        #krawedz laczaca powinna byc typu jednokierunkowego
        queueNodesId = [i for i in self.sourceNodes \
                      if self.sourceNodes[i]["type"] == baseNodeType["queue"]]
        for i in queueNodesId:
            inNodes = [self.reducedEdges[j]["sourceNodes"][0] for j in self.reducedEdges \
                       if self.reducedEdges[j]["sourceNodes"][-1] == i] 
            outNodes = [self.reducedEdges[j]["sourceNodes"][-1] for j in self.reducedEdges \
                        if self.reducedEdges[j]["sourceNodes"][0] == i]
            assert len(inNodes) == 1, "Only one intersection node should be connected as input with queue."
            assert len(outNodes) == 1, "Only one intersection node should be connected as output with queue."
            inNodeType = self.sourceNodes[inNodes[0]]["type"]
            outNodeType = self.sourceNodes[outNodes[0]]["type"]
            assert (inNodeType == baseNodeType["intersection"] and outNodeType == baseNodeType["intersection"]), "Connected Queue with given nodes not allowed. Available connection: intersection->queue->intersection."
            
            inEdgeType = [self.reducedEdges[j]["wayType"] for j in self.reducedEdges \
                          if self.reducedEdges[j]["sourceNodes"][0] == inNodes[0] \
                              and self.reducedEdges[j]["sourceNodes"][-1] == i]
            
            outEdgeType = [self.reducedEdges[j]["wayType"] for j in self.reducedEdges \
                          if self.reducedEdges[j]["sourceNodes"][0] == i \
                              and self.reducedEdges[j]["sourceNodes"][-1] == outNodes[0]]
            assert inEdgeType[0] == wayType["oneWay"] \
                and outEdgeType[0] == wayType["oneWay"], "Edges should be one way in connection intersection->queue->intersection."
    
    def validateWaitingConnectionNodes(self):
        #pobranie wszystkich miejsc w ktorych roboty oczekuja na dojazd do stanowiska (waiting)
        #sprawdzenie czy wezel poprzedzajacy jest skrzyżowaniem, a następny stanowiskiem
        #krawedz laczaca powinna byc typu jednokierunkowego
        queueNodesId = [i for i in self.sourceNodes \
                      if self.sourceNodes[i]["type"] == baseNodeType["waiting"]]
        for i in queueNodesId:
            inNodes = [self.reducedEdges[j]["sourceNodes"][0] for j in self.reducedEdges \
                       if self.reducedEdges[j]["sourceNodes"][-1] == i] 
            outNodes = [self.reducedEdges[j]["sourceNodes"][-1] for j in self.reducedEdges \
                        if self.reducedEdges[j]["sourceNodes"][0] == i]
            assert len(inNodes) == 1, "Only one intersection node should be connected as input with waiting node."
            assert len(outNodes) == 1, "Only one POI node should be connected as output with waiting node."
            inNodeType = self.sourceNodes[inNodes[0]]["type"]
            outNodeType = self.sourceNodes[outNodes[0]]["type"]["nodeSection"]
            assert (inNodeType == baseNodeType["intersection"] and 
                    (outNodeType == baseNodeSectionType["dockWaitUndock"] or \
                     outNodeType == baseNodeSectionType["waitPOI"])), "Connected waiting node with given nodes not allowed.Available connection:  intersection->waiting->POI."
            inEdgeType = [self.reducedEdges[j]["wayType"] for j in self.reducedEdges \
                          if self.reducedEdges[j]["sourceNodes"][0] == inNodes[0] \
                              and self.reducedEdges[j]["sourceNodes"][-1] == i]
            
            outEdgeType = [self.reducedEdges[j]["wayType"] for j in self.reducedEdges \
                          if self.reducedEdges[j]["sourceNodes"][0] == i \
                              and self.reducedEdges[j]["sourceNodes"][-1] == outNodes[0]]
            assert inEdgeType[0] == wayType["oneWay"] \
                    and outEdgeType[0] == wayType["oneWay"], "Edges should be one way in connection intersection->waiting->POI." 
         
    def validateDepartureConnectionNodes(self):
        #pobranie wszystkich miejsc w ktorych roboty odjeżdżają od stanowiska (departure)
        #sprawdzenie czy wezel poprzedzajacy stanowiskiem, a następny skrzyżowaniem
        #krawedz laczaca powinna byc typu jednokierunkowego
        queueNodesId = [i for i in self.sourceNodes \
                      if self.sourceNodes[i]["type"] == baseNodeType["departure"]]
        for i in queueNodesId:
            inNodes = [self.reducedEdges[j]["sourceNodes"][0] for j in self.reducedEdges \
                       if self.reducedEdges[j]["sourceNodes"][-1] == i] 
            outNodes = [self.reducedEdges[j]["sourceNodes"][-1] for j in self.reducedEdges \
                        if self.reducedEdges[j]["sourceNodes"][0] == i]
            assert len(inNodes) == 1, "Only one POI node should be connected as input with departure node."
            assert len(outNodes) == 1, "Only one departure node should be connected as output with intersection node."
            inNodeType = self.sourceNodes[inNodes[0]]["type"]["nodeSection"]
            outNodeType = self.sourceNodes[outNodes[0]]["type"]
            assert (outNodeType == baseNodeType["intersection"] and 
                    (inNodeType == baseNodeSectionType["dockWaitUndock"] or \
                     inNodeType == baseNodeSectionType["waitPOI"])), "Connected departure node with given nodes not allowed. Available connection: POI->departure->intersection."
            inEdgeType = [self.reducedEdges[j]["wayType"] for j in self.reducedEdges \
                          if self.reducedEdges[j]["sourceNodes"][0] == inNodes[0] \
                              and self.reducedEdges[j]["sourceNodes"][-1] == i]
            
            outEdgeType = [self.reducedEdges[j]["wayType"] for j in self.reducedEdges \
                          if self.reducedEdges[j]["sourceNodes"][0] == i \
                              and self.reducedEdges[j]["sourceNodes"][-1] == outNodes[0]]
            assert inEdgeType[0] == wayType["oneWay"] \
                    and outEdgeType[0] == wayType["oneWay"], "Edges should be one way in connection intersection->queue->intersection." 
    
    def validateWaitDepConnectionNodes(self):
        #TODO dokonczyzc funkcje + przetestowac dzialanie funkcji i zwracanie bledow w przypadkach testowych
        #pobranie wszystkich miejsc w ktorych roboty podjeżdżają/odjeżdżają od stanowiska (waiting-departure node)
        #dla polaczenia:
        #intersection-> waiting-departure -> POI krawedzie sa typu dwukierunkowa szeroka oraz dwukierunkowa waska
        #POI -> waiting-departure -> intersection krawedzie sa typu dwukierunkowa waska oraz dwukierunkowa szeroka
        queueNodesId = [i for i in self.sourceNodes \
                      if self.sourceNodes[i]["type"] == baseNodeType["waiting-departure"]]
        for i in queueNodesId:
            inNodes = [self.reducedEdges[j]["sourceNodes"][0] for j in self.reducedEdges \
                       if self.reducedEdges[j]["sourceNodes"][-1] == i] 
            outNodes = [self.reducedEdges[j]["sourceNodes"][-1] for j in self.reducedEdges \
                        if self.reducedEdges[j]["sourceNodes"][0] == i]
            assert len(inNodes) == 2 and len(outNodes) == 2, "Too much nodes connected with witing-departure node."

            inNodeType = [self.sourceNodes[inNodes[0]]["type"], self.sourceNodes[inNodes[1]]["type"]]
            outNodeType = [self.sourceNodes[outNodes[0]]["type"], self.sourceNodes[outNodes[1]]["type"]]
            inNodeTypeSection = [self.sourceNodes[inNodes[0]]["type"]["nodeSection"], \
                          self.sourceNodes[inNodes[1]]["type"]["nodeSection"]]
            outNodeTypeSection = [self.sourceNodes[outNodes[0]]["type"]["nodeSection"],\
                          self.sourceNodes[outNodes[1]]["type"]["nodeSection"]]  
            
            assert (baseNodeType["intersection"] in inNodeType) \
                    and (baseNodeSectionType["dockWaitUndock"] in inNodeTypeSection or baseNodeSectionType["waitPOI"] in inNodeTypeSection) \
                    and (baseNodeType["intersection"] in outNodeType) \
                    and (baseNodeSectionType["dockWaitUndock"] in outNodeTypeSection or baseNodeSectionType["waitPOI"] in outNodeTypeSection),  \
                        "Connected waiting-departure node with given nodes not allowed. Node should be connected with intersection and POI."
            #powstale krawedzie z polaczen powinny byc odpowiednio dwukierunkowe szerokie oraz waskie dwukierunkowe
            interWaitDepPathNodes = []
            poiWaitDepPathNodes = []
            if inNodeType[0] == baseNodeType["intersection"]:
                #pierwszy wezel jest skrzyzowaniem
                interWaitDepPathNodes.append([inNodes[0],i])
                poiWaitDepPathNodes.append([inNodes[1],i])
            else:
                #drugi wezel jest skrzyzowaniem
                interWaitDepPathNodes.append([inNodes[1],i])
                poiWaitDepPathNodes.append([inNodes[0],i])
            
            if outNodeType[0] == baseNodeType["intersection"]:
                #pierwszy wezel jest skrzyzowaniem
                interWaitDepPathNodes.append([i, outNodes[0]])
                poiWaitDepPathNodes.append([i,outNodes[1]])
            else:
                #drugi wezel jest skrzyzowaniem
                interWaitDepPathNodes.append([i, outNodes[1]])
                poiWaitDepPathNodes.append([i,outNodes[0]])              
                
            #sprawdzenie typu krawedzi laczacych sie z waiting departure
            interWaitDepPathType = []
            poiWaitDepPathType = []
            for j in self.reducedEdges:
                edge = self.reducedEdges[j]
                for path in interWaitDepPathNodes:
                    if edge["sourceNodes"][0] == path[0] and edge["sourceNodes"][-1] == path[1]:  
                        interWaitDepPathType.append(edge["wayType"])
                for path in poiWaitDepPathNodes:
                    if edge["sourceNodes"][0] == path[0] and edge["sourceNodes"][-1] == path[1]:  
                        poiWaitDepPathType.append(edge["wayType"])
                        
            assert interWaitDepPathType[0] == wayType["twoWay"] \
                and interWaitDepPathType[1] == wayType["twoWay"], \
                "Edges should be twoWay in connection intersection<->waiting-departure." 
            assert poiWaitDepPathType[0] == wayType["narrowTwoWay"] \
                and poiWaitDepPathType[1] == wayType["narrowTwoWay"], \
                "Edges should be twoNarrowWay in connection waiting-departure<->POI"     
            
    def getGraphData(self):
        return self.reducedEdges
    
    def printGraphData(self):
        for i in self.reducedEdges:
            print(i, self.reducedEdges[i])

class SupervisorGraphCreator(DataConverter):
    def __init__(self, sourceNodes, sourceEdges):
        # odczyt danych, wywołanie odpowiednich funkcji i utworzenie właściwego grafu
        self.graph = nx.DiGraph()
        self.graphNodeId = 1
        self.edgeGroupId = 1
        self.edgeId = 1
        self.groupIdSwitcher = {}
        self.convertDataRun(sourceNodes, sourceEdges)
        self.createGraph()
    
    def createGraph(self):
        combinedEdges = self.setGroups(self.reducedEdges)
        self.addPoiDockingNodes()
        self.addPoiNoDocking()
        self.addPoiNoChanges()
        self.addMainPath(combinedEdges)
        self.addIntersetionsPath()
        self.addNodesPosition(combinedEdges)  
        self.assignPoiToWaitingEdges()
        self.setDefaultTimeWeight()
        self.setMaxRobots()
    
    def setGroups(self, combinedEdges):
        poiParkingNodeIds = [i for i in self.sourceNodes \
                             if self.sourceNodes[i]["type"]["nodeSection"] in [baseNodeSectionType["dockWaitUndock"],baseNodeSectionType["waitPOI"]] \
                                or self.sourceNodes[i]["type"] == baseNodeType["parking"]]
        for i in poiParkingNodeIds:
            self.groupIdSwitcher[i] = self.edgeGroupId
            self.edgeGroupId = self.edgeGroupId + 1                                
        
        for i in combinedEdges:
            edge = combinedEdges[i]
            groupId = 0
            if edge["sourceNodes"][0] in poiParkingNodeIds:
                groupId = self.groupIdSwitcher[edge["sourceNodes"][0]]
            elif edge["sourceNodes"][-1] in poiParkingNodeIds:
                groupId = self.groupIdSwitcher[edge["sourceNodes"][-1]]
            combinedEdges[i]["edgeGroupId"] = groupId
            combinedEdges[i]["sourceEdges"] = edge["sourceEdges"]
        #pobranie waskich dwukierunkowych drog ktore nie sa przypisane do grupy
        narrowWaysId = [i for i in combinedEdges \
                        if combinedEdges[i]["wayType"] == wayType["narrowTwoWay"] \
                           and combinedEdges[i]["edgeGroupId"] == 0]
        if len(narrowWaysId) > 0:
            while True:
                pathId = next(iter(narrowWaysId))  
                currentPath = combinedEdges[pathId]["sourceNodes"]
                twinPath = currentPath[::-1]
                narrowWaysId.remove(pathId)
                for i in narrowWaysId:
                    if twinPath == combinedEdges[i]["sourceNodes"]:
                        narrowWaysId.remove(i) 
                        combinedEdges[pathId]["edgeGroupId"] = self.edgeGroupId
                        combinedEdges[i]["edgeGroupId"] = self.edgeGroupId
                        self.edgeGroupId = self.edgeGroupId + 1
                        break
                if len(narrowWaysId) == 0:
                    break

        return combinedEdges        
        
    def getAllNodesByTypeSection(self, givenType):
        nodes = [(i, self.sourceNodes[i]) for i in self.sourceNodes if self.sourceNodes[i]["type"]["nodeSection"] == givenType]
        return nodes

    def addPoiDockingNodes(self):
        dockNodes = self.getAllNodesByTypeSection(baseNodeSectionType["dockWaitUndock"])
        for dockNode in dockNodes:
            nodeId = dockNode[0]
            sourceNode = dockNode[1]
            self.graph.add_node(self.graphNodeId, nodeType = newNodeType["dock"], sourceNode=nodeId, 
                                color=nodeColor["dock"], poiId=dockNode[1]["poiId"])
            self.graph.add_edge(self.graphNodeId,self.graphNodeId+1,id = self.edgeId,  weight=0, behaviour=Task.behType["dock"],\
                                edgeGroupId = self.groupIdSwitcher[nodeId], sourceNodes=[nodeId], sourceEdges = [0])
            self.graphNodeId = self.graphNodeId + 1
            self.edgeId = self.edgeId + 1
            self.graph.add_node(self.graphNodeId, nodeType = newNodeType["wait"], sourceNode=nodeId, 
                                color=nodeColor["wait"], poiId = dockNode[1]["poiId"])
            self.graph.add_edge(self.graphNodeId,self.graphNodeId+1, id = self.edgeId,  weight=0, behaviour=Task.behType["wait"],\
                                edgeGroupId = self.groupIdSwitcher[nodeId], sourceNodes=[nodeId], sourceEdges = [0])
            self.graphNodeId = self.graphNodeId + 1 
            self.edgeId = self.edgeId + 1
            self.graph.add_node(self.graphNodeId, nodeType = newNodeType["undock"], sourceNode=nodeId,
                                color=nodeColor["undock"], poiId = dockNode[1]["poiId"]) 
            self.graph.add_edge(self.graphNodeId,self.graphNodeId+1, id = self.edgeId,  weight=0, behaviour=Task.behType["undock"],\
                                edgeGroupId = self.groupIdSwitcher[nodeId], sourceNodes=[nodeId], sourceEdges = [0])
            self.graphNodeId = self.graphNodeId + 1
            self.edgeId = self.edgeId + 1
            self.graph.add_node(self.graphNodeId, nodeType = newNodeType["end"], sourceNode=nodeId, 
                                color=nodeColor["end"], poiId = dockNode[1]["poiId"])        
            self.graphNodeId = self.graphNodeId + 1
 
    def addPoiNoDocking(self):
        noDockNodes = self.getAllNodesByTypeSection(baseNodeSectionType["waitPOI"])
        for noDockNode in noDockNodes:
            nodeId = noDockNode[0]
            sourceNode = noDockNode[1]
            self.graph.add_node(self.graphNodeId, nodeType = newNodeType["wait"], sourceNode=nodeId, 
                                color=nodeColor["wait"], poiId = noDockNode[1]["poiId"])
            self.graph.add_edge(self.graphNodeId,self.graphNodeId+1,id = self.edgeId,  weight=0, behaviour=Task.behType["wait"],\
                                edgeGroupId = self.groupIdSwitcher[nodeId], sourceNodes=[nodeId], sourceEdges = [0])
            self.graphNodeId = self.graphNodeId + 1

            self.graph.add_node(self.graphNodeId, nodeType = newNodeType["end"], sourceNode=nodeId, 
                                color=nodeColor["end"], poiId = noDockNode[1]["poiId"])        
            self.graphNodeId = self.graphNodeId + 1

    def addPoiNoChanges(self):
        nodes = self.getAllNodesByTypeSection(baseNodeSectionType["noChanges"])
        for node in nodes:
            nodeId = node[0]
            sourceNode = node[1]
            self.graph.add_node(self.graphNodeId, nodeType = newNodeType["noChanges"], sourceNode=nodeId, 
                                color=nodeColor["noChanges"], poiId = node[1]["poiId"])
            self.graphNodeId = self.graphNodeId + 1
###############################################################################################################    
    def addMainPath(self, combinedEdges):         
        for i in combinedEdges:
            sourceNodeId = combinedEdges[i]["sourceNodes"]
            #print(i, combinedEdges[i])
            if self.sourceNodes[sourceNodeId[0]]["type"]["nodeSection"] == baseNodeSectionType["intersection"] and \
                self.sourceNodes[sourceNodeId[-1]]["type"]["nodeSection"] == baseNodeSectionType["intersection"]:
                #oba wezly sa skrzyowaniami, mozna od razu utworzyc docelowa krawedz grafu laczaca je
                self.graph.add_node(self.graphNodeId, nodeType = newNodeType["intersection_out"], \
                                    sourceNode=sourceNodeId[0], color=nodeColor["out"], poiId = 0)
                self.graphNodeId = self.graphNodeId + 1    
                self.graph.add_node(self.graphNodeId, nodeType = newNodeType["intersection_in"], \
                                    sourceNode=sourceNodeId[-1], color=nodeColor["in"], poiId = 0)
                self.graph.add_edge(self.graphNodeId-1,self.graphNodeId, id = self.edgeId, weight=0, \
                                    behaviour=Task.behType["goto"], edgeGroupId = combinedEdges[i]["edgeGroupId"], \
                                    wayType = combinedEdges[i]["wayType"], sourceNodes=sourceNodeId, \
                                    sourceEdges = combinedEdges[i]["sourceEdges"])
                self.graphNodeId = self.graphNodeId + 1
                self.edgeId = self.edgeId + 1
            elif self.sourceNodes[sourceNodeId[-1]]["type"]["nodeSection"] == baseNodeSectionType["intersection"]and \
                self.sourceNodes[sourceNodeId[0]]["type"]["nodeSection"] != baseNodeSectionType["intersection"]:
                #wezel koncowy krawedzi jest typu intersection
                self.graph.add_node(self.graphNodeId, nodeType = newNodeType["intersection_in"], \
                                    sourceNode=sourceNodeId[-1], color=nodeColor["in"], poiId = 0)
                self.graphNodeId = self.graphNodeId + 1
                gNodeId = self.getConnectedGraphNodeId(sourceNodeId[0])
                self.graph.add_edge(gNodeId,self.graphNodeId-1, id = self.edgeId, weight=0, behaviour=Task.behType["goto"],\
                                edgeGroupId = combinedEdges[i]["edgeGroupId"], \
                                    wayType = combinedEdges[i]["wayType"], sourceNodes=sourceNodeId, \
                                    sourceEdges = combinedEdges[i]["sourceEdges"])
                self.edgeId = self.edgeId + 1
            elif self.sourceNodes[sourceNodeId[0]]["type"]["nodeSection"] == baseNodeSectionType["intersection"]and \
                self.sourceNodes[sourceNodeId[-1]]["type"]["nodeSection"] != baseNodeSectionType["intersection"]:
                #wezel poczatkowy krawedzi jest typu intersection
                self.graph.add_node(self.graphNodeId, nodeType = newNodeType["intersection_out"], \
                                    sourceNode=sourceNodeId[0], color=nodeColor["out"], poiId = 0)
                self.graphNodeId = self.graphNodeId + 1
                gNodeId = self.getConnectedGraphNodeId(sourceNodeId[-1], edgeStartNode = False)
                self.graph.add_edge(self.graphNodeId-1,gNodeId, id = self.edgeId, weight=0, behaviour=Task.behType["goto"],\
                                edgeGroupId = combinedEdges[i]["edgeGroupId"], \
                                wayType = combinedEdges[i]["wayType"], sourceNodes=sourceNodeId, \
                                sourceEdges = combinedEdges[i]["sourceEdges"])
                self.edgeId = self.edgeId + 1
            else:
                startNodeId = self.getConnectedGraphNodeId(sourceNodeId[0])
                endNodeId = self.getConnectedGraphNodeId(sourceNodeId[-1], edgeStartNode = False)
                self.graph.add_edge(startNodeId,endNodeId, id = self.edgeId, weight=0, behaviour=Task.behType["goto"],\
                                edgeGroupId = combinedEdges[i]["edgeGroupId"], \
                                wayType = combinedEdges[i]["wayType"], sourceNodes=sourceNodeId, \
                                sourceEdges = combinedEdges[i]["sourceEdges"])  
                self.edgeId = self.edgeId + 1
    
    def getConnectedGraphNodeId(self, sourceNodeId, edgeStartNode = True):
        graphNodeId = None
        data = [(n, v)for n,v in self.graph.nodes(data=True) if v["sourceNode"] == sourceNodeId] 
        nodeType = self.sourceNodes[sourceNodeId]["type"]["nodeSection"]
        if nodeType == baseNodeSectionType["dockWaitUndock"]:
            if edgeStartNode:
                #zakonczono operacje na stanowisku
                end = [node for node,v in data if v["nodeType"] == newNodeType["end"]] 
                graphNodeId = end[0]
            else:
                #operacja na stanowisku zostanie rozpoczeta
                start = [node for node,v in data if v["nodeType"] == newNodeType["dock"]] 
                graphNodeId = start[0]
        elif nodeType == baseNodeSectionType["waitPOI"]:
            if edgeStartNode:
                #zakonczono operacje na stanowisku
                end = [node for node,v in data if v["nodeType"] == newNodeType["end"]] 
                graphNodeId = end[0]
            else:
                #operacja na stanowisku zostanie rozpoczeta
                start = [node for node,v in data if v["nodeType"] == newNodeType["wait"]] 
                graphNodeId = start[0]        
        elif nodeType == baseNodeSectionType["noChanges"]:
                start = [node for node,v in data] 
                graphNodeId = start[0]          
        else:
            graphNodeId = None
        return graphNodeId
        
    def addIntersetionsPath(self):
        intersections = self.getAllNodesByTypeSection(baseNodeSectionType["intersection"])
        operationPois = [i for i in self.sourceNodes \
                            if self.sourceNodes[i]["type"]["nodeSection"] \
                            in [baseNodeSectionType["dockWaitUndock"], baseNodeSectionType["waitPOI"]]]
                                                 
        for intersection in intersections:    
            i = intersection[0]
            waitDepIntersection = False
            groupId = self.edgeGroupId
            nodeIn = [node for node,v in self.graph.nodes(data=True) \
                      if v["sourceNode"] == i and v["nodeType"]== newNodeType["intersection_in"]]
            nodeOut = [node for node,v in self.graph.nodes(data=True) \
                      if v["sourceNode"] == i and v["nodeType"]== newNodeType["intersection_out"]]
            all_combinations = list(itertools.product(nodeIn, nodeOut))    
            
            if self.sourceNodes[i]["type"] == baseNodeType["waiting-departure"]:
                #wezel dla ktorego budowane jest skrzyzowanie jest wezlem oczekiwania
                # krawedzie zwiazane z tym skrzyzowaniem powinny nalezec do stanowiska
                connectedEdges = [mainEdge for mainEdge in self.graph.edges(data=True) \
                                  if (mainEdge[2]["sourceNodes"][0] in operationPois or \
                                      mainEdge[2]["sourceNodes"][-1] in operationPois)]            
                startNode = connectedEdges[0][2]["sourceNodes"][0]
                endNode = connectedEdges[0][2]["sourceNodes"][-1]
                poiSourceNode = startNode if startNode in operationPois else endNode
                groupId = self.groupIdSwitcher[poiSourceNode]
                waitDepIntersection = True
            if len(all_combinations) != 0:                
                for edge in all_combinations:
                    self.graph.add_edge(edge[0],edge[1],id = self.edgeId,weight=0, behaviour=Task.behType["goto"],\
                                    edgeGroupId = groupId, wayType = wayType["oneWay"], \
                                    sourceNodes=[i], sourceEdges = [0])
                    self.edgeId = self.edgeId + 1
                if not waitDepIntersection:
                    self.edgeGroupId = self.edgeGroupId + 1

    def assignPoiToWaitingEdges(self):    
        #startWaitingNode - punkt w ktorym ustawi sie pierwszy robot w kolejce do czekania, grot strzalki, koniec krawedzi
        #endWaitingNode - punkt w ktorym zakolejkuje sie ostatni robot, koncowka krawedzi wzdluz ktorej ustawiaja 
        #pobranie id wezlow bazowych dla parking, queue
        waitingNodes = [i for i in self.sourceNodes if self.sourceNodes[i]["type"] in [baseNodeType["parking"],baseNodeType["queue"]]] 
        waitingGraphNodes = [node for node in self.graph.nodes(data=True) if node[1]["sourceNode"] in waitingNodes]
        for node in waitingGraphNodes:
            #wezel grafu do ktorego punkt jest przypisany
            startWaitingNode = node[0]
            #wyznaczenie wezla poprzedzajacego
            endWaitingNode = [edge[0] for edge in self.graph.edges(data=True) if edge[1] == startWaitingNode][0]
            #dopisanie do krawedzi o poczatku w wezle poprzedzajacym i koncu w wezle odniesienia grafu id
            #poi typu parking, queue
            sourceNodeId = node[1]["sourceNode"]
            self.graph.edges[endWaitingNode,startWaitingNode]["connectedPoi"] = self.sourceNodes[sourceNodeId]["poiId"]
 
        #pobranie id wezlow bazowych waiting
        poiWaitNodes = [i for i in self.sourceNodes if self.sourceNodes[i]["type"] == baseNodeType["waiting"]]
        poiWaitGraphNodes = [node for node in self.graph.nodes(data=True) if node[1]["sourceNode"] in poiWaitNodes]
        for node in poiWaitGraphNodes:                
            #wezel grafu do ktorego punkt jest przypisany          
            startWaitingNode = node[0]           
            #wyznaczenie wezla poprzedzajacego
            endWaitingNode = [edge[0] for edge in self.graph.edges(data=True) if edge[1] == startWaitingNode][0]
            connectedPoi = [edge[1] for edge in self.graph.edges(data=True) if edge[0] == startWaitingNode][0]
            graphPoiNode = self.graph.nodes[connectedPoi]["sourceNode"]
            self.graph.edges[endWaitingNode, startWaitingNode]["connectedPoi"] = self.sourceNodes[graphPoiNode]["poiId"]
          
        
        #pobranie id wezlow bazowych waiting-departure 
        poiWaitDepNodes = [i for i in self.sourceNodes if self.sourceNodes[i]["type"] == baseNodeType["waiting-departure"]]     
        for nodeId in poiWaitDepNodes:  
           #print("startWaitingNode", startWaitingNode)
            graphNodeIds = [node[0] for node in self.graph.nodes(data=True)\
                           if node[1]["sourceNode"] == nodeId \
                            and node[1]["nodeType"] == newNodeType["intersection_in"]]
            #powinny byc dwa wezly, jeden laczy sie ze stanowiskiem, a drugi z wezlem
            #skrzyzowania i wchodzi w sklad krawedzi na ktorej kolejkują się roboty
            edges = [edge for edge in self.graph.edges(data=True)\
                              if edge[1] in graphNodeIds]
            #powinny zostac znalezione dwie krawedzie spelniajace wymagania
            #rint("edges", edges)
            assert len(edges) == 2, "waiting path, departure-waiting poi error"
            startWaitingNode = None
            endWaitingNode = None 
            sourceNode = None
            if edges[0][2]["edgeGroupId"] != 0:
                #pierwsza zapisana krawedz zwiazana jest ze stanowiskiem
                #druga dotyczy oczekiwania
                startWaitingNode = edges[1][1]
                endWaitingNode = edges[1][0]
                sourceId = self.graph.nodes[edges[0][0]]["sourceNode"]
            else:
                #druga zapisana krawedz dotyczy stanowiska, a pierwsza oczekiwania
                startWaitingNode = [0][1]
                endWaitingNode = edges[0][0]
                sourceId = self.graph.nodes[edges[1][0]]["sourceNode"]
            self.graph.edges[endWaitingNode, startWaitingNode]["connectedPoi"] = self.sourceNodes[sourceId]["poiId"]
                
    def addNodesPosition(self, combinedEdges):
        for nodeId,nodeData in self.graph.nodes(data=True):
            nodePosition = self.sourceNodes[nodeData["sourceNode"]]["pos"]
            nodeType = nodeData["nodeType"]
            if nodeType == newNodeType["dock"]:
                self.graph.nodes[nodeId]["pos"] = self.getPoiNodesPos(nodeId,combinedEdges)
            elif nodeType == newNodeType["wait"]:
                self.graph.nodes[nodeId]["pos"] = self.getPoiNodesPos(nodeId,combinedEdges)
            elif nodeType == newNodeType["undock"]:
                self.graph.nodes[nodeId]["pos"] = self.getPoiNodesPos(nodeId,combinedEdges)
            elif nodeType == newNodeType["end"]:
                self.graph.nodes[nodeId]["pos"] = self.getPoiNodesPos(nodeId,combinedEdges)
            elif nodeType == newNodeType["noChanges"]:
                self.graph.nodes[nodeId]["pos"] = nodePosition
            elif nodeType == newNodeType["intersection_in"]:
                position = self.getNewIntersectionNodePosition(nodeId)
                self.graph.nodes[nodeId]["pos"] = position
            elif nodeType == newNodeType["intersection_out"]:
                position = self.getNewIntersectionNodePosition(nodeId) 
                self.graph.nodes[nodeId]["pos"] = position
                
    def getPoiNodesPos(self, graphNodeId, combinedEdges):
        #poszukiwanie id wezlow przed i za stanowiskiem na podstawie krawedzi grafu        
        sourceNodePoi = self.graph.nodes[graphNodeId]
        sourceNodePoiId = sourceNodePoi["sourceNode"]
        
        a = [data["sourceNodes"][0] for data in combinedEdges.values() \
             if data["sourceNodes"][-1] == sourceNodePoiId]
        
        nodeBeforePoiId = a[0]
        b = [data["sourceNodes"][-1] for data in combinedEdges.values() \
             if data["sourceNodes"][0] == sourceNodePoiId]
        
        nodeAfterPoiId = b[0]        
        nodeBeforePoiPos = self.sourceNodes[nodeBeforePoiId]["pos"]
        nodeAfterPoiPos = self.sourceNodes[nodeAfterPoiId]["pos"]  
        pA = [nodeBeforePoiPos[0],nodeBeforePoiPos[1]]
        pB = [nodeAfterPoiPos[0],nodeAfterPoiPos[1]]

        baseAngle = math.radians(np.rad2deg(np.arctan2(pB[1] - pA[1], pB[0]-pA[0])))
        distance = math.sqrt(math.pow(pA[0]-pB[0],2)+math.pow(pA[1]-pB[1],2))
        translateToBaseNode = np.array([[1, 0, 0, nodeBeforePoiPos[0]],
                      [0, 1, 0, nodeBeforePoiPos[1]],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        rotationToWay = np.array([[math.cos(baseAngle), -math.sin(baseAngle), 0, 0],
                      [math.sin(baseAngle), math.cos(baseAngle), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        step = distance/5
        nodesVect = []
        poiStepTranslation = np.array([[1, 0, 0, step*sourceNodePoi["nodeType"]],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])  
        nodeTranslation = np.dot(np.dot(translateToBaseNode,rotationToWay),poiStepTranslation)
        if self.sourceNodes[nodeBeforePoiId]["type"] == baseNodeType["waiting-departure"]:
            sId = self.graph.nodes[graphNodeId]["sourceNode"]
            nodesVect.append([self.sourceNodes[sId]["pos"][0], self.sourceNodes[sId]["pos"][1]])
        else:
            nodesVect.append([nodeTranslation[0][3],nodeTranslation[1][3]])
        
        return (nodesVect[0][0],nodesVect[0][1])
    
    def getNewIntersectionNodePosition(self, graphNodeId):
        #A - punkt startowy krawedzi
        #B - punkt koncowy krawedzi
      # print("------------------------------")
        pA = [0,0]
        pB = [0,0]
        nodeType = self.graph.nodes[graphNodeId]["nodeType"]
        sourceNodeId = self.graph.nodes[graphNodeId]["sourceNode"]
        mainGraphNodesId = []
        for i in self.graph.edges:
            if i[0] == graphNodeId:
                mainGraphNodesId.append(i[1])
            elif i[1] == graphNodeId:
                mainGraphNodesId.append(i[0])
                              
        endGraphNodeId = 0
        endSourceNodeId = None
        for i in mainGraphNodesId:
            if self.graph.nodes[i]["sourceNode"] != sourceNodeId:
                endSourceNodeId = self.graph.nodes[i]["sourceNode"]
                endGraphNodeId = i
        pA = [self.sourceNodes[sourceNodeId]["pos"][0],self.sourceNodes[sourceNodeId]["pos"][1]]   
        
        pathType = [edgeType[2]["wayType"] for edgeType in self.graph.edges(data=True) \
                    if (edgeType[0] == graphNodeId and edgeType[1] == endGraphNodeId) or
                       (edgeType[1] == graphNodeId and edgeType[0] == endGraphNodeId)]
        if endSourceNodeId != None:
            position = [0,0]
            graphSourceNodes = [edge[2]["sourceNodes"] for edge in self.graph.edges(data=True) \
                                if (edge[0] == graphNodeId and edge[1] == endGraphNodeId)
                or (edge[0] == endGraphNodeId and edge[1] == graphNodeId)]
            if len(graphSourceNodes[0]) >= 2:   
                if graphSourceNodes[0][0] == sourceNodeId:
                    position = self.sourceNodes[graphSourceNodes[0][1]]["pos"]
                else:
                    position = self.sourceNodes[graphSourceNodes[0][-2]]["pos"]
            else:
                    position = self.sourceNodes[endSourceNodeId]["pos"]
            pB = [position[0],position[1]]
            baseAngle = math.radians(np.rad2deg(np.arctan2(pB[1] - pA[1], pB[0]-pA[0])))
            translateToBaseNode = np.array([[1, 0, 0, pA[0]],
                          [0, 1, 0, pA[1]],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

            rotationToWay = np.array([[math.cos(baseAngle), -math.sin(baseAngle), 0, 0],
                          [math.sin(baseAngle), math.cos(baseAngle), 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

            wayNode = np.array([[1, 0, 0, corridorWidth + robotLength],
                          [0, 1, 0, corridorWidth/2],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

            if nodeType == newNodeType["intersection_out"]:
                wayNode[1][3] = -corridorWidth/2
            
            if len(pathType) != 0:
                if pathType[0] != wayType["twoWay"]:
                    wayNode[1][3] = 0
            way = np.dot(np.dot(translateToBaseNode,rotationToWay),wayNode)
            return (way[0][3],way[1][3])
        else:
            return (0,0)
        
    def setDefaultTimeWeight(self):
        robotVelocity = 0.5 # [m/s]
        for i in self.graph.edges:
            edge = self.graph.edges[i]
            isBlocked = False
            for eId in edge["sourceEdges"]:
                if eId != 0:
                    if self.sourceEdges[eId]["isActive"]:
                        isBlocked = True
                        break
            if isBlocked:
                 self.graph.edges[i]["weight"] = blockWeight
            elif edge["behaviour"] == Task.behType["goto"] and edge["edgeGroupId"] != 0 \
                and len(edge["sourceNodes"]) == 1:
                self.graph.edges[i]["weight"] = 3        
            elif edge["behaviour"] == Task.behType["goto"]:    
                nodesPos = []
                for nodeId in edge["sourceNodes"]:
                    nodesPos.append(self.sourceNodes[nodeId]["pos"])
                dist = 0        
                for j in range(len(nodesPos)-1):
                    dist = dist + math.hypot(nodesPos[j+1][0] - nodesPos[j][0], nodesPos[j+1][1] - nodesPos[j][1])
                self.graph.edges[i]["weight"] = math.ceil(dist/robotVelocity)                  
            elif edge["behaviour"] == Task.behType["dock"]:
                self.graph.edges[i]["weight"] = 5
            elif edge["behaviour"] == Task.behType["wait"]:
                self.graph.edges[i]["weight"] = 8
            elif edge["behaviour"] == Task.behType["undock"]:
                self.graph.edges[i]["weight"] = 4
    
    def setMaxRobots(self):
        operationPois = [i for i in self.sourceNodes \
                            if self.sourceNodes[i]["type"]["nodeSection"] \
                            in [baseNodeSectionType["dockWaitUndock"], baseNodeSectionType["waitPOI"]]
                            or self.sourceNodes[i]["type"] == baseNodeType["parking"]] 
        for i in self.graph.edges:
            edge = self.graph.edges[i]
            noPoiNodes = not (edge["sourceNodes"][0] in operationPois or edge["sourceNodes"][-1] in operationPois)
            if edge["behaviour"] == Task.behType["goto"] and len(edge["sourceNodes"]) != 1 and noPoiNodes:
                nodesPos = []
                for nodeId in edge["sourceNodes"]:
                    nodesPos.append(self.sourceNodes[nodeId]["pos"])
                dist = 0        
                for j in range(len(nodesPos)-1):
                    dist = dist + math.hypot(nodesPos[j+1][0] - nodesPos[j][0], nodesPos[j+1][1] - nodesPos[j][1])
                self.graph.edges[i]["maxRobots"] = math.floor(dist/robotLength)

    def getCorridorPath(self, edgeId, sourcePath):
        #sourcePath = [(x,y),(x2,y2),...]
        line = LineString(sourcePath)
        corridor = line.buffer(corridorWidth/2,cap_style=3, join_style=2)
        unia = unary_union([corridor])
        x, y = unia.exterior.coords.xy
        sourcePathCorridor = [(x[i],y[i]) for i in range(len(x))]
        startNodePos = [self.graph.nodes[edge[0]]["pos"] for edge in self.graph.edges(data=True) \
                        if edge[2]["id"] == edgeId][0]
        endNodePos = [self.graph.nodes[edge[1]]["pos"] for edge in self.graph.edges(data=True)\
                      if edge[2]["id"] == edgeId][0]
        itemToDel = int((len(sourcePathCorridor) - 3)/2)
        del sourcePathCorridor[0:itemToDel]
        del sourcePathCorridor[-3:]
        sourcePathCorridor = sourcePathCorridor[::-1]
        sourcePathCorridor[0] = startNodePos
        sourcePathCorridor[-1] = endNodePos
        return sourcePathCorridor    
    
    def createCorridorCoordinates(self, robotPathCoordinates):
        #robotPathCoordinates = [(x,y),(x2,y2),...]
        #create corridor along the path
        line = LineString(robotPathCoordinates)
        corridor = line.buffer(corridorWidth/2,cap_style=3, join_style=2)
        
        poiA = self.getNodeArea(True, robotPathCoordinates)
        poiB = self.getNodeArea(False, robotPathCoordinates)
        
        #create final corridor
        unia = unary_union([poiB, corridor, poiA])
        x, y = unia.exterior.coords.xy
        finalCorridorCoordinates = [(x[i],y[i]) for i in range(len(x))]
        #finalCorridorCoordinates = [(x,y),(x2,y2),...]
        return finalCorridorCoordinates  
    
    def getNodeArea(self, start, robotPathCoordinates):
        goal = robotPathCoordinates[0] if start else robotPathCoordinates[-1]
        orientPoint = robotPathCoordinates[1] if start else robotPathCoordinates[-2]

        x = goal[0] - orientPoint[0]
        y = goal[1] - orientPoint[1]
        radius = math.sqrt(x * x + y * y )
        theta = math.atan2(y,x)
        new_x = (radius + 0.01) * math.cos(theta)
        new_y = (radius + 0.01) * math.sin(theta)
        
        #nowy punkt w ukladzie mapy
        newPoint = (new_x + orientPoint[0], new_y + orientPoint[1])
        poi = LineString([[goal[0],goal[1]],[newPoint[0],newPoint[1]]]).buffer(corridorWidth, cap_style=3,join_style=2)
        return poi    
    
    def getCorridorCoordinates(self, edgeId):
        #zadania musza byc typu go to aby mozna bylo skorzystac
        edge = [data for data in self.graph.edges(data=True) if data[2]["id"] == edgeId][0]
       # print(edge)
       # print(len(edge["sourceNodes"]))
        assert Task.behType["goto"] == edge[2]["behaviour"], "Corridors can be only created to 'goto' behaviur edge type"
        #obsluga dla skrzyzowan
        if len(edge[2]["sourceNodes"]) == 1:
          #  print("skrzyzowanie")
            sourcePos = self.sourceNodes[edge[2]["sourceNodes"][0]]["pos"] 
            #wezel startowy
            startNodePos = [self.graph.nodes[data[0]]["pos"] for data in self.graph.edges(data=True)\
                            if data[2]["id"] == edgeId][0]
          #  print(np.arctan2(corridorWidth+robotLength, corridorWidth/2))
          #  print("source pos", sourcePos)
          #  print("start node", startNodePos)
          # # print(np.arctan2(sourcePos[1] - startNodePos[1], sourcePos[0] - startNodePos[0]))
            angle = np.arctan2(corridorWidth+robotLength, corridorWidth/2) + np.arctan2(sourcePos[1] - startNodePos[1], sourcePos[0] - startNodePos[0])
            translateToBaseNode = np.array([[1, 0, 0, startNodePos[0]],[0, 1, 0, startNodePos[1]], \
                          [0, 0, 1, 0], [0, 0, 0, 1]])
            rotationStartNode = np.array([[math.cos(angle), -math.sin(angle), 0, 0], \
                          [math.sin(angle), math.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            corridorStart = np.array([[1, 0, 0, corridorWidth/2], \
                          [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])                        
            way = np.dot(np.dot(translateToBaseNode,rotationStartNode),corridorStart)
            startPos = (way[0][3],way[1][3])
            
            #wezel koncowy
            endNodePos = [self.graph.nodes[data[1]]["pos"] for data in self.graph.edges(data=True) \
                          if data[2]["id"] == edgeId][0]
            angle2 = np.arctan2(corridorWidth/2,corridorWidth+robotLength) + np.arctan2(endNodePos[1] - sourcePos[1],endNodePos[0] -  sourcePos[0])
            translateToIntersection = np.array([[1, 0, 0, sourcePos[0]],[0, 1, 0, sourcePos[1]], \
                          [0, 0, 1, 0], [0, 0, 0, 1]])
            rotationToWay2 = np.array([[math.cos(angle2), -math.sin(angle2), 0, 0], \
                          [math.sin(angle2), math.cos(angle2), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            wayNode2 = np.array([[1, 0, 0, corridorWidth+robotLength], \
                          [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])                        
            way2 = np.dot(np.dot(translateToIntersection,rotationToWay2),wayNode2)
            endPos = (way2[0][3],way2[1][3])      
            
          #  startDifferentEdge = [data for data in self.graph.edges(data=True)\
          #      if data[1] == edge[0] and data[2]["wayType"] != edgeDirectionType["twoWay"]]
          #  if len(startDifferentEdge) > 0:
          #      startPos = startNodePos
                
          #  endDifferentEdge = [data for data in self.graph.edges(data=True)\
         #       if data[0] == edge[1] and data[2]["wayType"] != edgeDirectionType["twoWay"]]
         #   if len(endDifferentEdge) > 0:
          #      endPos = endNodePos
          #  print("start len", startDifferentEdge, "end len", endDifferentEdge)
            line = LineString([startPos, sourcePos, endPos])  
            corridor = line.buffer(corridorWidth/2,cap_style=3, join_style=2)
            unia = unary_union([corridor])
            x, y = unia.exterior.coords.xy
            baseCoordinates = [(x[i],y[i]) for i in range(len(x))]

            x_source = [startNodePos[0],sourcePos[0],endNodePos[0]]
            y_source = [startNodePos[1],sourcePos[1],endNodePos[1]]
            x_path = [point[0] for point in [startPos, sourcePos, endPos]]
            y_path = [point[1] for point in [startPos, sourcePos, endPos]]
            x_cor = [point[0] for point in baseCoordinates]
            y_cor = [point[1] for point in baseCoordinates]
            plt.plot(x_source,y_source,"g",x_path,y_path,"r",x_cor,y_cor,"b")
            #plt.plot(x_source,y_source,"g",x_path,y_path,"r")
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.show() 
            return baseCoordinates
        else: 
            print("inne krawedzie")
            #inne krawedzie
            sourceId = [data[2]["sourceNodes"] for data in self.graph.edges(data=True) \
                        if data[2]["id"] == edgeId][0]
            sourcePath = [self.sourceNodes[i]["pos"] for i in sourceId]
            pathCoordinates = self.getCorridorPath(edgeId, sourcePath)
            return self.createCorridorCoordinates(pathCoordinates)
        
    def printCorridor(self, edgeId):
        #pobranie wspolrzednych wezlow z grafu wlasciwego
        sourceId = [data[2]["sourceNodes"] for data in self.graph.edges(data=True) if data[2]["id"] == edgeId][0]
        sourcePath = [self.sourceNodes[i]["pos"] for i in sourceId]
        #przesuniecie i wygenerowanie lamanej po ktorej bedzie poruszal sie robot
        pathCoordinates = self.getCorridorPath(edgeId, sourcePath)
        #wygenerowanie korytarza         
        corridorCoordinates = self.getCorridorCoordinates(edgeId)
        #wyswietlenie korytarza
        x_source = [point[0] for point in sourcePath]
        y_source = [point[1] for point in sourcePath]
        x_path = [point[0] for point in pathCoordinates]
        y_path = [point[1] for point in pathCoordinates]
        x_cor = [point[0] for point in corridorCoordinates]
        y_cor = [point[1] for point in corridorCoordinates]
        plt.plot(x_source,y_source,"g",x_path,y_path,"r",x_cor,y_cor,"b")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.show()    
    
    def getGraph(self):
        return self.graph
    
    def printGraph(self, plotSize = (45,45)):
        plt.figure(figsize=plotSize)
        node_pos=nx.get_node_attributes(self.graph,"pos")
        #maxRobots=nx.get_edge_attributes(G,"maxRobots")
        #edge_col = [G[u][v]["color"] for u,v in G.edges()]
        node_col = [self.graph.nodes[i]["color"] for i in self.graph.nodes()]
        nx.draw_networkx(self.graph, node_pos,node_color = node_col, node_size=550,font_size=15,with_labels=True,font_color="w", width=2)
        #nx.draw_networkx(G, node_pos,edge_color= edge_col, node_color = node_col, node_size=3000,font_size=25,with_labels=True,font_color="w", width=4)
        #nx.draw_networkx_edge_labels(G, node_pos, edge_color= edge_col, node_color = node_col, edge_labels=maxRobots,font_size=30)
        plt.show()
        plt.close()

