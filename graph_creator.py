import networkx as nx
import numpy as np
import copy
import itertools
import math
import matplotlib.pyplot as plt  # do wizualizaccji i rysowania grafu
import json
from shapely.geometry import LineString
from shapely.ops import unary_union
from dispatcher import Task

corridor_width = 0.3  # 1
robot_length = 0.4  # 0.6
docking_time_weight = 20
undocking_time_weight = 20
wait_weight = 10

way_type = {
    "twoWay": 1,
    "narrowTwoWay": 2,
    "oneWay": 3
}
base_node_section_type = {
    "dockWaitUndock": 1,
    # rozbicie na dock, wait, undock, end (przesunąć współrzędne węzłów tylko do wyświetlania i diagnostyki,
    # docelowo można pominąć)
    "waitPOI": 2,
    # rozbicie na wait, end (przesunąć współrzędne węzłów tylko do wyświetlania i diagnostyki, docelowo można pominąć)
    "noChanges": 3,  # brak zmian, przepisać
    "normal": 4,  # pomijane przy rozbijaniu węzłów, ale uwzględniane przy kształcie korytarza
    "intersection": 5
    # rozbicie na in, out - liczba zależy od krawędzi związanych z węzłem - konieczność wygenerowania
    # prawidłowych współrzędnych przesunięcia
}
base_node_type = {
    "charger": {"id": 1, "nodeSection": base_node_section_type["dockWaitUndock"]},  # charger
    "load": {"id": 2, "nodeSection": base_node_section_type["waitPOI"]},  # loading POI
    "unload": {"id": 3, "nodeSection": base_node_section_type["waitPOI"]},  # unloading POI
    "load-unload": {"id": 4, "nodeSection": base_node_section_type["waitPOI"]},  # loading-unloading POI
    "load-dock": {"id": 5, "nodeSection": base_node_section_type["dockWaitUndock"]},  # loading-docking POI
    "unload-dock": {"id": 6, "nodeSection": base_node_section_type["dockWaitUndock"]},  # unloading-docking POI
    "load-unload-dock": {"id": 7, "nodeSection": base_node_section_type["dockWaitUndock"]},
    # loading-unloading-docking POI
    "waiting": {"id": 8, "nodeSection": base_node_section_type["noChanges"]},  # waiting POI
    "departure": {"id": 9, "nodeSection": base_node_section_type["noChanges"]},  # departue POI
    "waiting-departure": {"id": 10, "nodeSection": base_node_section_type["intersection"]},  # waiting-departure-POI
    "parking": {"id": 11, "nodeSection": base_node_section_type["noChanges"]},  # parking POI
    "queue": {"id": 12, "nodeSection": base_node_section_type["noChanges"]},  # queue POI
    "normal": {"id": 13, "nodeSection": base_node_section_type["normal"]},  # normal POI
    "intersection": {"id": 14, "nodeSection": base_node_section_type["intersection"]},  # intersection POI
}
new_node_type = {
    # type - służy do przyszłego złączenia krawędzi grafu w odpowiedniej kolejności
    # znaczenie typow wezlow
    "dock": 1,  # rozpoczecie dokowania
    "wait": 2,  # rozpoczecie wlaciwej operacji na stanowisku
    "undock": 3,  # rozpoczecie procedury oddokowania
    "end": 4,  # zakonczenie zadania, mozliwosc przekierowania robota do innego zadania (wolny) lub kolejnego zachowania
    "noChanges": 5,  # wezel nie ulega zmianom wzgledem tego od ktorego jest tworzony
    "intersection_in": 6,  # wjazd na skrzyzowanie
    "intersection_out": 7  # zjazd ze skrzyzowania
}
node_color = {  # do celow wizualizacji i podgladu
    "dockWaitUndock": "yellow",  # L
    "dock": "#00fc00",  # jasna zielen
    "wait": "red",
    "undock": "#00a800",  # ciemna zielen
    "end": "#8c0000",  # ciemny czerwony
    "noChanges": "orange",  # S
    "intersection_base": "blue",  # D
    "in": "black",  # P
    "out": "magenta",  # WP
    "skipped": "#72fdfe"  # blękit - wezly ktore powinny byc pominiete, ale chyba beda rysowane
}


def convert_database_nodes(source_nodes):
    """
    Konwersja danych z bazy danych do obslugiwanego formatu przez klasy.
    sourceNodes
    {id: {"name": string,
          "pos": (x,y),
          "type": GraphDataConverter.baseNodeType["type"],
          "poiID": int}, ...}
    """
    database_nodes = json.loads(source_nodes)
    converted_nodes = {}
    for node in database_nodes:
        converted_nodes[node["id"]] = {"name": node["name"], "pos": (node["posX"], node["posY"]),
                                       "type": node["type"], "poiID": node["poiID"]}
    return converted_nodes


def convert_database_edges(source_edges):
    """
    Konwersja danych z bazy danych do obslugiwanego formatu przez klasy.
    sourceEdges
    {id: {"startNode": string,
          "endNode"
          "type":  GraphDataConverter.wayType["type"],
          }, ...}
    """
    database_edges = json.loads(source_edges)
    converted_edges = {}
    for edge in database_edges:
        if edge["biDirected"] and edge["narrow"]:
            edge_way_type = way_type["twoWay"]
        elif edge["biDirected"] and (edge["narrow"] is False):
            edge_way_type = way_type["narrowTwoWay"]
        else:
            edge_way_type = way_type["oneWay"]
        converted_edges[edge["id"]] = {
            "startNode": edge["vertexB"]["id"],
            "endNode": edge["vertexA"]["id"],
            "type": edge_way_type,
            "isActive": edge["isActive"]
        }
    return converted_edges


def create_edges(source_edges):
    i = 1
    extended_edges = {}
    for k in source_edges:
        edge = source_edges[k]
        if edge["type"] == way_type["twoWay"] or \
                edge["type"] == way_type["narrowTwoWay"]:

            a = edge["startNode"]
            b = edge["endNode"]
            extended_edges[i] = edge.copy()
            extended_edges[i]["startNode"] = a
            extended_edges[i]["endNode"] = b
            extended_edges[i]["edgeSource"] = k
            i = i + 1
            extended_edges[i] = edge.copy()
            extended_edges[i]["startNode"] = b
            extended_edges[i]["endNode"] = a
            extended_edges[i]["edgeSource"] = k
            i = i + 1
            pass
        else:
            extended_edges[i] = edge.copy()
            extended_edges[i]["edgeSource"] = k
            i = i + 1
    return extended_edges


class DataConverter:
    def __init__(self, source_nodes, source_edges):
        self.source_nodes = source_nodes
        self.source_edges = source_edges
        edges = create_edges(copy.deepcopy(source_edges))
        self.reduced_edges = self.combined_normal_nodes(edges)
        self.convert_data_run()

    def convert_data_run(self):
        # odczyt danych, wywołanie odpowiednich funkcji i utworzenie właściwego grafu
        # TODO odkomentowac 2 ponizsze linie jestli na werjsciu maja byc dane z grafu odczytanego z bazy odpowiednio
        #  dla wezlow i krawedzi
        # self.source_nodes = convert_database_nodes(source_nodes)
        # self.source_edges = convert_database_edges(source_edges)
        # na potrzeby testow
        # TODO zakomentowac 2 ponizsze linie jest wkorzystane maja byc jako dane wejsciowe dane z bazy danych
        # self.source_nodes = source_nodes
        # self.source_edges = source_edges
        # edges = create_edges(copy.deepcopy(source_edges))
        # self.reduced_edges = self.combined_normal_nodes(edges)
        self.validate_poi_connection_nodes()
        self.validate_parking_connection_nodes()
        self.validate_queue_connection_nodes()
        self.validate_waiting_connection_nodes()
        self.validate_departure_connection_nodes()
        self.validate_wait_dep_connection_nodes()
        print("dane prawidłowe")

    def combined_normal_nodes(self, edges):
        normal_nodes_id = {i for i in self.source_nodes if self.source_nodes[i]["type"] == base_node_type["normal"]}
        combined_path = {edgeId: {"edgeSourceId": [edges[edgeId]["edgeSource"]],
                                  "connectedNodes": [edges[edgeId]["startNode"], edges[edgeId]["endNode"]]}
                         for edgeId in edges if edges[edgeId]["startNode"] not in normal_nodes_id
                         and edges[edgeId]["endNode"] in normal_nodes_id}
        edges_normal_nodes = [i for i in edges if edges[i]["startNode"] in normal_nodes_id]
        previos_edge_len = len(edges_normal_nodes)
        while True:
            for i in edges_normal_nodes:
                edge = edges[i]
                for j in combined_path:
                    node = combined_path[j]
                    if node["connectedNodes"][-1] == edge["startNode"] and \
                            edge["edgeSource"] not in node["edgeSourceId"]:
                        combined_path[j]["connectedNodes"].append(edge["endNode"])
                        combined_path[j]["edgeSourceId"].append(edge["edgeSource"])
                        edges_normal_nodes.remove(i)
                        break

            if previos_edge_len == len(edges_normal_nodes):
                assert len(
                    edges_normal_nodes) == 0, "Normal nodes error. Path contains normal nodes should start and end " \
                                              "from different type of node."
                break
            else:
                previos_edge_len = len(edges_normal_nodes)
        self._validate_direction_combined_path(combined_path)
        return self._combined_normal_nodes_edges(edges, combined_path)

    def _validate_direction_combined_path(self, connected_normal_nodes_paths):
        combined_path = copy.deepcopy(connected_normal_nodes_paths)
        for i in combined_path:
            edges_id = combined_path[i]["edgeSourceId"]
            previous_path_type = self.source_edges[edges_id[0]]["type"]
            for j in range(len(edges_id) - 1):
                current_path_type = self.source_edges[edges_id[j + 1]]["type"]
                assert previous_path_type == current_path_type, "Different path type connected to normal nodes."
            end_node_id = combined_path[i]["connectedNodes"][-1]
            end_node_type = self.source_nodes[end_node_id]["type"]
            assert end_node_type != base_node_type[
                "normal"], "Path contains normal nodes should be end different type of nodes."

    def _combined_normal_nodes_edges(self, edges, combined_path):
        # utworzenie listy krawędzi z pominięciem wezlow normalnych do dalszych analiz polaczen miedzy
        # typami wezlow
        reduced_edges = {}
        j = 1
        for i in edges:
            edge = edges[i]
            if self.source_nodes[edge["startNode"]]["type"] != base_node_type["normal"] \
                    and self.source_nodes[edge["endNode"]]["type"] != base_node_type["normal"]:
                if j not in reduced_edges:
                    reduced_edges[j] = {"sourceEdges": [], "sourceNodes": [], "wayType": 0}
                reduced_edges[j]["sourceEdges"] = [edge["edgeSource"]]
                reduced_edges[j]["sourceNodes"] = [edge["startNode"], edge["endNode"]]
                reduced_edges[j]["wayType"] = edge["type"]
                j = j + 1
            elif self.source_nodes[edge["startNode"]]["type"] != base_node_type["normal"] \
                    and self.source_nodes[edge["endNode"]]["type"] == base_node_type["normal"]:
                last_node_id = [pathId for pathId in combined_path
                                if combined_path[pathId]["connectedNodes"][0] == edge["startNode"]
                                and combined_path[pathId]["connectedNodes"][1] == edge["endNode"]]
                if j not in reduced_edges:
                    reduced_edges[j] = {"sourceEdges": [], "sourceNodes": [], "wayType": 0}
                reduced_edges[j]["sourceEdges"] = combined_path[last_node_id[0]]["edgeSourceId"]
                reduced_edges[j]["sourceNodes"] = combined_path[last_node_id[0]]["connectedNodes"]
                reduced_edges[j]["wayType"] = edge["type"]
                del combined_path[last_node_id[0]]
                j = j + 1
        return reduced_edges

    def validate_poi_connection_nodes(self):
        # pobranie wszystkich poi z dokowaniem i bez dokowania
        # sprawdzenie czy kazde POI laczy sie z dwoma innymi wezlami
        # - konfiguracja I waiting Poi, departure poi
        # - walidacja krawedzi obie sa jednokierunkowe, inaczej blad typu krawedzi
        # - konfiguracja II waiting-departure poi, poi waiting departure
        # - walidacja krawedzi - powinna byc typu waskiego dwukierunkowa inaczej blad typu krawedzi
        # dla wszystkich innych polaczen blad grafu i polaczen stanowisk
        poi_nodes_id = [i for i in self.source_nodes
                        if self.source_nodes[i]["type"]["nodeSection"] == base_node_section_type["dockWaitUndock"]
                        or self.source_nodes[i]["type"]["nodeSection"] == base_node_section_type["waitPOI"]]
        for i in poi_nodes_id:
            in_nodes = [self.reduced_edges[j]["sourceNodes"][0] for j in self.reduced_edges
                        if self.reduced_edges[j]["sourceNodes"][-1] == i]
            out_nodes = [self.reduced_edges[j]["sourceNodes"][-1] for j in self.reduced_edges
                         if self.reduced_edges[j]["sourceNodes"][0] == i]
            assert len(in_nodes) == 1, "Only one waiting/waiting-departure POI should be connected with POI."
            assert len(out_nodes) == 1, "Only one departure/waiting-departure POI should be connected with POI."

            in_node_type = self.source_nodes[in_nodes[0]]["type"]
            out_node_type = self.source_nodes[out_nodes[0]]["type"]

            assert (in_node_type == base_node_type["waiting"] and out_node_type == base_node_type["departure"]) \
                   or (in_node_type == base_node_type["waiting-departure"] and
                       out_node_type == base_node_type["waiting-departure"]), "Connected POI with given nodes not " \
                                                                              "allowed. Available connection: " \
                                                                              "waiting->POI->departure or" \
                                                                              " waiting-departure->POI->" \
                                                                              "waiting-departure"

            in_edge_type = [self.reduced_edges[j]["wayType"] for j in self.reduced_edges
                            if self.reduced_edges[j]["sourceNodes"][0] == in_nodes[0]
                            and self.reduced_edges[j]["sourceNodes"][-1] == i]

            out_edge_type = [self.reduced_edges[j]["wayType"] for j in self.reduced_edges
                             if self.reduced_edges[j]["sourceNodes"][0] == i
                             and self.reduced_edges[j]["sourceNodes"][-1] == out_nodes[0]]
            if in_node_type == base_node_type["waiting"]:
                assert in_edge_type[0] == way_type["oneWay"] and \
                       out_edge_type[0] == way_type[
                           "oneWay"], "Edges should be one way in connection waiting->POI->departure"
            else:
                assert in_edge_type[0] == way_type["narrowTwoWay"] and \
                       out_edge_type[0] == way_type[
                           "narrowTwoWay"], "Edges should be narrow two way in connection " \
                                            "waiting-departure->POI->waiting-departure"

    def validate_parking_connection_nodes(self):
        # pobranie wszystkich miejsc parkingowych
        # sprawdzenie czy parking laczy sie bezposrednio ze skrzyzowaniem
        # krawedz laczaca powinna byc typu waskieg dwukierunkowego
        parking_nodes_id = [i for i in self.source_nodes
                            if self.source_nodes[i]["type"] == base_node_type["parking"]]
        for i in parking_nodes_id:
            in_nodes = [self.reduced_edges[j]["sourceNodes"][0] for j in self.reduced_edges
                        if self.reduced_edges[j]["sourceNodes"][-1] == i]
            out_nodes = [self.reduced_edges[j]["sourceNodes"][-1] for j in self.reduced_edges
                         if self.reduced_edges[j]["sourceNodes"][0] == i]
            assert len(in_nodes) == 1, "Only one intersection node should be connected as input with parking."
            assert len(out_nodes) == 1, "Only one intersection node should be connected as output with parking."
            in_node_type = self.source_nodes[in_nodes[0]]["type"]
            out_node_type = self.source_nodes[out_nodes[0]]["type"]
            assert (in_node_type == base_node_type["intersection"] and out_node_type == base_node_type[
                "intersection"]), "Connected Parking with given nodes not allowed. Available connection: " \
                                  "intersection->POI->intersection."

            in_edge_type = [self.reduced_edges[j]["wayType"] for j in self.reduced_edges
                            if self.reduced_edges[j]["sourceNodes"][0] == in_nodes[0]
                            and self.reduced_edges[j]["sourceNodes"][-1] == i]

            out_edge_type = [self.reduced_edges[j]["wayType"] for j in self.reduced_edges
                             if self.reduced_edges[j]["sourceNodes"][0] == i
                             and self.reduced_edges[j]["sourceNodes"][-1] == out_nodes[0]]
            assert in_edge_type[0] == way_type["narrowTwoWay"] and \
                   out_edge_type[0] == way_type[
                       "narrowTwoWay"], "Edges should be narrow two way in connection" \
                                        " intersection->parking->intersection."

    def validate_queue_connection_nodes(self):
        # pobranie wszystkich miejsc w ktorych kolejkowane beda roboty do parkowania
        # sprawdzenie czy wezel poprzedzajacy i nastepny lacza sie ze skrzyzowaniem
        # krawedz laczaca powinna byc typu jednokierunkowego
        queue_nodes_id = [i for i in self.source_nodes
                          if self.source_nodes[i]["type"] == base_node_type["queue"]]
        for i in queue_nodes_id:
            in_nodes = [self.reduced_edges[j]["sourceNodes"][0] for j in self.reduced_edges
                        if self.reduced_edges[j]["sourceNodes"][-1] == i]
            out_nodes = [self.reduced_edges[j]["sourceNodes"][-1] for j in self.reduced_edges
                         if self.reduced_edges[j]["sourceNodes"][0] == i]
            assert len(in_nodes) == 1, "Only one intersection node should be connected as input with queue."
            assert len(out_nodes) == 1, "Only one intersection node should be connected as output with queue."
            in_node_type = self.source_nodes[in_nodes[0]]["type"]
            out_node_type = self.source_nodes[out_nodes[0]]["type"]
            assert (in_node_type == base_node_type["intersection"] and out_node_type == base_node_type[
                "intersection"]), "Connected Queue with given nodes not allowed. Available connection: " \
                                  "intersection->queue->intersection."

            in_edge_type = [self.reduced_edges[j]["wayType"] for j in self.reduced_edges
                            if self.reduced_edges[j]["sourceNodes"][0] == in_nodes[0]
                            and self.reduced_edges[j]["sourceNodes"][-1] == i]

            out_edge_type = [self.reduced_edges[j]["wayType"] for j in self.reduced_edges
                             if self.reduced_edges[j]["sourceNodes"][0] == i
                             and self.reduced_edges[j]["sourceNodes"][-1] == out_nodes[0]]
            assert in_edge_type[0] == way_type["oneWay"] \
                   and out_edge_type[0] == way_type[
                       "oneWay"], "Edges should be one way in connection intersection->queue->intersection."

    def validate_waiting_connection_nodes(self):
        # pobranie wszystkich miejsc w ktorych roboty oczekuja na dojazd do stanowiska (waiting)
        # sprawdzenie czy wezel poprzedzajacy jest skrzyżowaniem, a następny stanowiskiem
        # krawedz laczaca powinna byc typu jednokierunkowego
        queue_nodes_id = [i for i in self.source_nodes
                          if self.source_nodes[i]["type"] == base_node_type["waiting"]]
        for i in queue_nodes_id:
            in_nodes = [self.reduced_edges[j]["sourceNodes"][0] for j in self.reduced_edges
                        if self.reduced_edges[j]["sourceNodes"][-1] == i]
            out_nodes = [self.reduced_edges[j]["sourceNodes"][-1] for j in self.reduced_edges
                         if self.reduced_edges[j]["sourceNodes"][0] == i]
            assert len(in_nodes) == 1, "Only one intersection node should be connected as input with waiting node."
            assert len(out_nodes) == 1, "Only one POI node should be connected as output with waiting node."
            in_node_type = self.source_nodes[in_nodes[0]]["type"]
            out_node_type = self.source_nodes[out_nodes[0]]["type"]["nodeSection"]
            assert (in_node_type == base_node_type["intersection"] and
                    (out_node_type == base_node_section_type["dockWaitUndock"] or
                     out_node_type == base_node_section_type[
                         "waitPOI"])), "Connected waiting node with given nodes not allowed.Available connection: " \
                                       " intersection->waiting->POI."
            in_edge_type = [self.reduced_edges[j]["wayType"] for j in self.reduced_edges
                            if self.reduced_edges[j]["sourceNodes"][0] == in_nodes[0]
                            and self.reduced_edges[j]["sourceNodes"][-1] == i]

            out_edge_type = [self.reduced_edges[j]["wayType"] for j in self.reduced_edges
                             if self.reduced_edges[j]["sourceNodes"][0] == i
                             and self.reduced_edges[j]["sourceNodes"][-1] == out_nodes[0]]
            assert in_edge_type[0] == way_type["oneWay"] \
                   and out_edge_type[0] == way_type[
                       "oneWay"], "Edges should be one way in connection intersection->waiting->POI."

    def validate_departure_connection_nodes(self):
        # pobranie wszystkich miejsc w ktorych roboty odjeżdżają od stanowiska (departure)
        # sprawdzenie czy wezel poprzedzajacy stanowiskiem, a następny skrzyżowaniem
        # krawedz laczaca powinna byc typu jednokierunkowego
        queue_nodes_id = [i for i in self.source_nodes
                          if self.source_nodes[i]["type"] == base_node_type["departure"]]
        for i in queue_nodes_id:
            in_nodes = [self.reduced_edges[j]["sourceNodes"][0] for j in self.reduced_edges
                        if self.reduced_edges[j]["sourceNodes"][-1] == i]
            out_nodes = [self.reduced_edges[j]["sourceNodes"][-1] for j in self.reduced_edges
                         if self.reduced_edges[j]["sourceNodes"][0] == i]
            assert len(in_nodes) == 1, "Only one POI node should be connected as input with departure node."
            assert len(out_nodes) == 1, "Only one departure node should be connected as output with intersection node."
            in_node_type = self.source_nodes[in_nodes[0]]["type"]["nodeSection"]
            out_node_type = self.source_nodes[out_nodes[0]]["type"]
            assert (out_node_type == base_node_type["intersection"] and
                    (in_node_type == base_node_section_type["dockWaitUndock"] or
                     in_node_type == base_node_section_type[
                         "waitPOI"])), "Connected departure node with given nodes not allowed. Available connection: " \
                                       "POI->departure->intersection."
            in_edge_type = [self.reduced_edges[j]["wayType"] for j in self.reduced_edges
                            if self.reduced_edges[j]["sourceNodes"][0] == in_nodes[0]
                            and self.reduced_edges[j]["sourceNodes"][-1] == i]

            out_edge_type = [self.reduced_edges[j]["wayType"] for j in self.reduced_edges
                             if self.reduced_edges[j]["sourceNodes"][0] == i
                             and self.reduced_edges[j]["sourceNodes"][-1] == out_nodes[0]]
            assert in_edge_type[0] == way_type["oneWay"] \
                   and out_edge_type[0] == way_type[
                       "oneWay"], "Edges should be one way in connection intersection->queue->intersection."

    def validate_wait_dep_connection_nodes(self):
        # TODO dokonczyzc funkcje + przetestowac dzialanie funkcji i zwracanie bledow w przypadkach testowych
        # pobranie wszystkich miejsc w ktorych roboty podjeżdżają/odjeżdżają od stanowiska (waiting-departure node)
        # dla polaczenia:
        # intersection-> waiting-departure -> POI krawedzie sa typu dwukierunkowa szeroka oraz dwukierunkowa waska
        # POI -> waiting-departure -> intersection krawedzie sa typu dwukierunkowa waska oraz dwukierunkowa szeroka
        queue_nodes_id = [i for i in self.source_nodes
                          if self.source_nodes[i]["type"] == base_node_type["waiting-departure"]]
        for i in queue_nodes_id:
            in_nodes = [self.reduced_edges[j]["sourceNodes"][0] for j in self.reduced_edges
                        if self.reduced_edges[j]["sourceNodes"][-1] == i]
            out_nodes = [self.reduced_edges[j]["sourceNodes"][-1] for j in self.reduced_edges
                         if self.reduced_edges[j]["sourceNodes"][0] == i]
            assert len(in_nodes) == 2 and len(out_nodes) == 2, "Too much nodes connected with witing-departure node."

            in_node_type = [self.source_nodes[in_nodes[0]]["type"], self.source_nodes[in_nodes[1]]["type"]]
            out_node_type = [self.source_nodes[out_nodes[0]]["type"], self.source_nodes[out_nodes[1]]["type"]]
            in_node_type_section = [self.source_nodes[in_nodes[0]]["type"]["nodeSection"],
                                    self.source_nodes[in_nodes[1]]["type"]["nodeSection"]]
            out_node_type_section = [self.source_nodes[out_nodes[0]]["type"]["nodeSection"],
                                     self.source_nodes[out_nodes[1]]["type"]["nodeSection"]]

            assert (base_node_type["intersection"] in in_node_type) \
                   and (base_node_section_type["dockWaitUndock"] in in_node_type_section or base_node_section_type[
                        "waitPOI"] in in_node_type_section) \
                   and (base_node_type["intersection"] in out_node_type) \
                   and (base_node_section_type["dockWaitUndock"] in out_node_type_section or base_node_section_type[
                        "waitPOI"] in out_node_type_section), "Connected waiting-departure node with given nodes not " \
                                                              "allowed. Node should be connected with intersection " \
                                                              "and POI."
            # powstale krawedzie z polaczen powinny byc odpowiednio dwukierunkowe szerokie oraz waskie dwukierunkowe
            inter_wait_dep_path_nodes = []
            poi_wait_dep_path_nodes = []
            if in_node_type[0] == base_node_type["intersection"]:
                # pierwszy wezel jest skrzyzowaniem
                inter_wait_dep_path_nodes.append([in_nodes[0], i])
                poi_wait_dep_path_nodes.append([in_nodes[1], i])
            else:
                # drugi wezel jest skrzyzowaniem
                inter_wait_dep_path_nodes.append([in_nodes[1], i])
                poi_wait_dep_path_nodes.append([in_nodes[0], i])

            if out_node_type[0] == base_node_type["intersection"]:
                # pierwszy wezel jest skrzyzowaniem
                inter_wait_dep_path_nodes.append([i, out_nodes[0]])
                poi_wait_dep_path_nodes.append([i, out_nodes[1]])
            else:
                # drugi wezel jest skrzyzowaniem
                inter_wait_dep_path_nodes.append([i, out_nodes[1]])
                poi_wait_dep_path_nodes.append([i, out_nodes[0]])

                # sprawdzenie typu krawedzi laczacych sie z waiting departure
            inter_wait_dep_path_type = []
            poi_wait_dep_path_type = []
            for j in self.reduced_edges:
                edge = self.reduced_edges[j]
                for path in inter_wait_dep_path_nodes:
                    if edge["sourceNodes"][0] == path[0] and edge["sourceNodes"][-1] == path[1]:
                        inter_wait_dep_path_type.append(edge["wayType"])
                for path in poi_wait_dep_path_nodes:
                    if edge["sourceNodes"][0] == path[0] and edge["sourceNodes"][-1] == path[1]:
                        poi_wait_dep_path_type.append(edge["wayType"])

            assert inter_wait_dep_path_type[0] == way_type["twoWay"] \
                   and inter_wait_dep_path_type[1] == way_type["twoWay"], \
                   "Edges should be twoWay in connection intersection<->waiting-departure."
            assert poi_wait_dep_path_type[0] == way_type["narrowTwoWay"] \
                   and poi_wait_dep_path_type[1] == way_type["narrowTwoWay"], \
                   "Edges should be twoNarrowWay in connection waiting-departure<->POI"

    def get_graph_data(self):
        return self.reduced_edges

    def print_graph_data(self):
        for i in self.reduced_edges:
            print(i, self.reduced_edges[i])


def get_node_area(start, robot_path_coordinates):
    goal = robot_path_coordinates[0] if start else robot_path_coordinates[-1]
    orient_point = robot_path_coordinates[1] if start else robot_path_coordinates[-2]

    x = goal[0] - orient_point[0]
    y = goal[1] - orient_point[1]
    radius = math.sqrt(x * x + y * y)
    theta = math.atan2(y, x)
    new_x = (radius + 0.01) * math.cos(theta)
    new_y = (radius + 0.01) * math.sin(theta)

    # nowy punkt w ukladzie mapy
    new_point = (new_x + orient_point[0], new_y + orient_point[1])
    poi = LineString([[goal[0], goal[1]], [new_point[0], new_point[1]]]).buffer(corridor_width, cap_style=3,
                                                                                join_style=2)
    return poi


def create_corridor_coordinates(robot_path_coordinates):
    # robotPathCoordinates = [(x,y),(x2,y2),...]
    # create corridor along the path
    line = LineString(robot_path_coordinates)
    corridor = line.buffer(corridor_width / 2, cap_style=3, join_style=2)

    poi_a = get_node_area(True, robot_path_coordinates)
    poi_b = get_node_area(False, robot_path_coordinates)

    # create final corridor
    unia = unary_union([poi_b, corridor, poi_a])
    x, y = unia.exterior.coords.xy
    final_corridor_coordinates = [(x[i], y[i]) for i in range(len(x))]
    # finalCorridorCoordinates = [(x,y),(x2,y2),...]
    return final_corridor_coordinates


class SupervisorGraphCreator(DataConverter):
    def __init__(self, source_nodes, source_edges):
        # odczyt danych, wywołanie odpowiednich funkcji i utworzenie właściwego grafu
        super().__init__(source_nodes, source_edges)
        self.graph = nx.DiGraph()
        self.graph_node_id = 1
        self.edge_group_id = 1
        self.edge_id = 1
        self.group_id_switcher = {}
        self.convert_data_run()
        self.create_graph()

    def create_graph(self):
        combined_edges = self.set_groups(self.reduced_edges)
        self.add_poi_docking_nodes()
        self.add_poi_no_docking()
        self.add_poi_no_changes()
        self.add_main_path(combined_edges)
        self.add_intersetions_path()
        self.add_nodes_position(combined_edges)
        self.assign_poi_to_waiting_edges()
        self.set_default_time_weight()
        self.set_max_robots()

    def set_groups(self, combined_edges):
        poi_parking_node_ids = [i for i in self.source_nodes if self.source_nodes[i]["type"]["nodeSection"] in
                                [base_node_section_type["dockWaitUndock"], base_node_section_type["waitPOI"]]
                                or self.source_nodes[i]["type"] == base_node_type["parking"]]
        for i in poi_parking_node_ids:
            self.group_id_switcher[i] = self.edge_group_id
            self.edge_group_id = self.edge_group_id + 1

        for i in combined_edges:
            edge = combined_edges[i]
            group_id = 0
            if edge["sourceNodes"][0] in poi_parking_node_ids:
                group_id = self.group_id_switcher[edge["sourceNodes"][0]]
            elif edge["sourceNodes"][-1] in poi_parking_node_ids:
                group_id = self.group_id_switcher[edge["sourceNodes"][-1]]
            combined_edges[i]["edgeGroupId"] = group_id
            combined_edges[i]["sourceEdges"] = edge["sourceEdges"]
        # pobranie waskich dwukierunkowych drog ktore nie sa przypisane do grupy
        narrow_ways_id = [i for i in combined_edges
                          if combined_edges[i]["wayType"] == way_type["narrowTwoWay"]
                          and combined_edges[i]["edgeGroupId"] == 0]
        if len(narrow_ways_id) > 0:
            while True:
                path_id = next(iter(narrow_ways_id))
                current_path = combined_edges[path_id]["sourceNodes"]
                twin_path = current_path[::-1]
                narrow_ways_id.remove(path_id)
                for i in narrow_ways_id:
                    if twin_path == combined_edges[i]["sourceNodes"]:
                        narrow_ways_id.remove(i)
                        combined_edges[path_id]["edgeGroupId"] = self.edge_group_id
                        combined_edges[i]["edgeGroupId"] = self.edge_group_id
                        self.edge_group_id = self.edge_group_id + 1
                        break
                if len(narrow_ways_id) == 0:
                    break

        return combined_edges

    def get_all_nodes_by_type_section(self, given_type):
        nodes = [(i, self.source_nodes[i]) for i in self.source_nodes if
                 self.source_nodes[i]["type"]["nodeSection"] == given_type]
        return nodes

    def add_poi_docking_nodes(self):
        dock_nodes = self.get_all_nodes_by_type_section(base_node_section_type["dockWaitUndock"])
        for dock_node in dock_nodes:
            node_id = dock_node[0]
            self.graph.add_node(self.graph_node_id, nodeType=new_node_type["dock"], sourceNode=node_id,
                                color=node_color["dock"], poiId=dock_node[1]["poiId"])
            self.graph.add_edge(self.graph_node_id, self.graph_node_id + 1, id=self.edge_id, weight=0,
                                behaviour=Task.beh_type["dock"],
                                edgeGroupId=self.group_id_switcher[node_id], sourceNodes=[node_id], sourceEdges=[0])
            self.graph_node_id = self.graph_node_id + 1
            self.edge_id = self.edge_id + 1
            self.graph.add_node(self.graph_node_id, nodeType=new_node_type["wait"], sourceNode=node_id,
                                color=node_color["wait"], poiId=dock_node[1]["poiId"])
            self.graph.add_edge(self.graph_node_id, self.graph_node_id + 1, id=self.edge_id, weight=0,
                                behaviour=Task.beh_type["wait"],
                                edgeGroupId=self.group_id_switcher[node_id], sourceNodes=[node_id], sourceEdges=[0])
            self.graph_node_id = self.graph_node_id + 1
            self.edge_id = self.edge_id + 1
            self.graph.add_node(self.graph_node_id, nodeType=new_node_type["undock"], sourceNode=node_id,
                                color=node_color["undock"], poiId=dock_node[1]["poiId"])
            self.graph.add_edge(self.graph_node_id, self.graph_node_id + 1, id=self.edge_id, weight=0,
                                behaviour=Task.beh_type["undock"],
                                edgeGroupId=self.group_id_switcher[node_id], sourceNodes=[node_id], sourceEdges=[0])
            self.graph_node_id = self.graph_node_id + 1
            self.edge_id = self.edge_id + 1
            self.graph.add_node(self.graph_node_id, nodeType=new_node_type["end"], sourceNode=node_id,
                                color=node_color["end"], poiId=dock_node[1]["poiId"])
            self.graph_node_id = self.graph_node_id + 1

    def add_poi_no_docking(self):
        no_dock_nodes = self.get_all_nodes_by_type_section(base_node_section_type["waitPOI"])
        for no_dock_node in no_dock_nodes:
            node_id = no_dock_node[0]
            self.graph.add_node(self.graph_node_id, nodeType=new_node_type["wait"], sourceNode=node_id,
                                color=node_color["wait"], poiId=no_dock_node[1]["poiId"])
            self.graph.add_edge(self.graph_node_id, self.graph_node_id + 1, id=self.edge_id, weight=0,
                                behaviour=Task.beh_type["wait"],
                                edgeGroupId=self.group_id_switcher[node_id], sourceNodes=[node_id], sourceEdges=[0])
            self.graph_node_id = self.graph_node_id + 1

            self.graph.add_node(self.graph_node_id, nodeType=new_node_type["end"], sourceNode=node_id,
                                color=node_color["end"], poiId=no_dock_node[1]["poiId"])
            self.graph_node_id = self.graph_node_id + 1

    def add_poi_no_changes(self):
        nodes = self.get_all_nodes_by_type_section(base_node_section_type["noChanges"])
        for node in nodes:
            node_id = node[0]
            self.graph.add_node(self.graph_node_id, nodeType=new_node_type["noChanges"], sourceNode=node_id,
                                color=node_color["noChanges"], poiId=node[1]["poiId"])
            self.graph_node_id = self.graph_node_id + 1

    ###############################################################################################################
    def add_main_path(self, combined_edges):
        for i in combined_edges:
            source_node_id = combined_edges[i]["sourceNodes"]
            # print(i, combinedEdges[i])
            if self.source_nodes[source_node_id[0]]["type"]["nodeSection"] == base_node_section_type["intersection"] \
                    and self.source_nodes[source_node_id[-1]]["type"]["nodeSection"] == base_node_section_type[
                    "intersection"]:
                # oba wezly sa skrzyowaniami, mozna od razu utworzyc docelowa krawedz grafu laczaca je
                self.graph.add_node(self.graph_node_id, nodeType=new_node_type["intersection_out"],
                                    sourceNode=source_node_id[0], color=node_color["out"], poiId=0)
                self.graph_node_id = self.graph_node_id + 1
                self.graph.add_node(self.graph_node_id, nodeType=new_node_type["intersection_in"],
                                    sourceNode=source_node_id[-1], color=node_color["in"], poiId=0)
                self.graph.add_edge(self.graph_node_id - 1, self.graph_node_id, id=self.edge_id, weight=0,
                                    behaviour=Task.beh_type["goto"], edgeGroupId=combined_edges[i]["edgeGroupId"],
                                    wayType=combined_edges[i]["wayType"], sourceNodes=source_node_id,
                                    sourceEdges=combined_edges[i]["sourceEdges"])
                self.graph_node_id = self.graph_node_id + 1
                self.edge_id = self.edge_id + 1
            elif self.source_nodes[source_node_id[-1]]["type"]["nodeSection"] == base_node_section_type[
                "intersection"] and \
                    self.source_nodes[source_node_id[0]]["type"]["nodeSection"] != \
                    base_node_section_type["intersection"]:
                # wezel koncowy krawedzi jest typu intersection
                self.graph.add_node(self.graph_node_id, nodeType=new_node_type["intersection_in"],
                                    sourceNode=source_node_id[-1], color=node_color["in"], poiId=0)
                self.graph_node_id = self.graph_node_id + 1
                g_node_id = self.get_connected_graph_node_id(source_node_id[0])
                self.graph.add_edge(g_node_id, self.graph_node_id - 1, id=self.edge_id, weight=0,
                                    behaviour=Task.beh_type["goto"],
                                    edgeGroupId=combined_edges[i]["edgeGroupId"],
                                    wayType=combined_edges[i]["wayType"], sourceNodes=source_node_id,
                                    sourceEdges=combined_edges[i]["sourceEdges"])
                self.edge_id = self.edge_id + 1
            elif self.source_nodes[source_node_id[0]]["type"]["nodeSection"] == base_node_section_type["intersection"] \
                    and self.source_nodes[source_node_id[-1]]["type"]["nodeSection"] != base_node_section_type[
                    "intersection"]:
                # wezel poczatkowy krawedzi jest typu intersection
                self.graph.add_node(self.graph_node_id, nodeType=new_node_type["intersection_out"],
                                    sourceNode=source_node_id[0], color=node_color["out"], poiId=0)
                self.graph_node_id = self.graph_node_id + 1
                g_node_id = self.get_connected_graph_node_id(source_node_id[-1], edge_start_node=False)
                self.graph.add_edge(self.graph_node_id - 1, g_node_id, id=self.edge_id, weight=0,
                                    behaviour=Task.beh_type["goto"],
                                    edgeGroupId=combined_edges[i]["edgeGroupId"],
                                    wayType=combined_edges[i]["wayType"], sourceNodes=source_node_id,
                                    sourceEdges=combined_edges[i]["sourceEdges"])
                self.edge_id = self.edge_id + 1
            else:
                start_node_id = self.get_connected_graph_node_id(source_node_id[0])
                end_node_id = self.get_connected_graph_node_id(source_node_id[-1], edge_start_node=False)
                self.graph.add_edge(start_node_id, end_node_id, id=self.edge_id, weight=0,
                                    behaviour=Task.beh_type["goto"], edgeGroupId=combined_edges[i]["edgeGroupId"],
                                    wayType=combined_edges[i]["wayType"], sourceNodes=source_node_id,
                                    sourceEdges=combined_edges[i]["sourceEdges"])
                self.edge_id = self.edge_id + 1

    def get_connected_graph_node_id(self, source_node_id, edge_start_node=True):
        data = [(n, v) for n, v in self.graph.nodes(data=True) if v["sourceNode"] == source_node_id]
        node_type = self.source_nodes[source_node_id]["type"]["nodeSection"]
        if node_type == base_node_section_type["dockWaitUndock"]:
            if edge_start_node:
                # zakonczono operacje na stanowisku
                end = [node for node, v in data if v["nodeType"] == new_node_type["end"]]
                return end[0]
            else:
                # operacja na stanowisku zostanie rozpoczeta
                start = [node for node, v in data if v["nodeType"] == new_node_type["dock"]]
                return start[0]
        elif node_type == base_node_section_type["waitPOI"]:
            if edge_start_node:
                # zakonczono operacje na stanowisku
                end = [node for node, v in data if v["nodeType"] == new_node_type["end"]]
                return end[0]
            else:
                # operacja na stanowisku zostanie rozpoczeta
                start = [node for node, v in data if v["nodeType"] == new_node_type["wait"]]
                return start[0]
        elif node_type == base_node_section_type["noChanges"]:
            start = [node for node, v in data]
            return start[0]
        else:
            return None

    def add_intersetions_path(self):
        intersections = self.get_all_nodes_by_type_section(base_node_section_type["intersection"])
        operation_pois = [i for i in self.source_nodes
                          if self.source_nodes[i]["type"]["nodeSection"]
                          in [base_node_section_type["dockWaitUndock"], base_node_section_type["waitPOI"]]]

        for intersection in intersections:
            i = intersection[0]
            wait_dep_intersection = False
            group_id = self.edge_group_id
            node_in = [node for node, v in self.graph.nodes(data=True)
                       if v["sourceNode"] == i and v["nodeType"] == new_node_type["intersection_in"]]
            node_out = [node for node, v in self.graph.nodes(data=True)
                        if v["sourceNode"] == i and v["nodeType"] == new_node_type["intersection_out"]]
            all_combinations = list(itertools.product(node_in, node_out))

            if self.source_nodes[i]["type"] == base_node_type["waiting-departure"]:
                # wezel dla ktorego budowane jest skrzyzowanie jest wezlem oczekiwania
                # krawedzie zwiazane z tym skrzyzowaniem powinny nalezec do stanowiska
                connected_edges = [main_edge for main_edge in self.graph.edges(data=True)
                                   if (main_edge[2]["sourceNodes"][0] in operation_pois or
                                       main_edge[2]["sourceNodes"][-1] in operation_pois)]
                start_node = connected_edges[0][2]["sourceNodes"][0]
                end_node = connected_edges[0][2]["sourceNodes"][-1]
                poi_source_node = start_node if start_node in operation_pois else end_node
                group_id = self.group_id_switcher[poi_source_node]
                wait_dep_intersection = True
            if len(all_combinations) != 0:
                for edge in all_combinations:
                    self.graph.add_edge(edge[0], edge[1], id=self.edge_id, weight=0, behaviour=Task.beh_type["goto"],
                                        edgeGroupId=group_id, wayType=way_type["oneWay"],
                                        sourceNodes=[i], sourceEdges=[0])
                    self.edge_id = self.edge_id + 1
                if not wait_dep_intersection:
                    self.edge_group_id = self.edge_group_id + 1

    def assign_poi_to_waiting_edges(self):
        # startWaitingNode - punkt w ktorym ustawi sie pierwszy robot w kolejce do czekania, grot strzalki,
        # koniec krawedzi
        # endWaitingNode - punkt w ktorym zakolejkuje sie ostatni robot, koncowka krawedzi wzdluz ktorej ustawiaja
        # pobranie id wezlow bazowych dla parking, queue
        waiting_nodes = [i for i in self.source_nodes if
                         self.source_nodes[i]["type"] in [base_node_type["parking"], base_node_type["queue"]]]
        waiting_graph_nodes = [node for node in self.graph.nodes(data=True) if node[1]["sourceNode"] in waiting_nodes]
        for node in waiting_graph_nodes:
            # wezel grafu do ktorego punkt jest przypisany
            start_waiting_node = node[0]
            # wyznaczenie wezla poprzedzajacego
            end_waiting_node = [edge[0] for edge in self.graph.edges(data=True) if edge[1] == start_waiting_node][0]
            # dopisanie do krawedzi o poczatku w wezle poprzedzajacym i koncu w wezle odniesienia grafu id
            # poi typu parking, queue
            source_node_id = node[1]["sourceNode"]
            self.graph.edges[end_waiting_node, start_waiting_node]["connectedPoi"] = self.source_nodes[source_node_id][
                "poiId"]

        # pobranie id wezlow bazowych waiting
        poi_wait_nodes = [i for i in self.source_nodes if self.source_nodes[i]["type"] == base_node_type["waiting"]]
        poi_wait_graph_nodes = [node for node in self.graph.nodes(data=True) if node[1]["sourceNode"] in poi_wait_nodes]
        for node in poi_wait_graph_nodes:
            # wezel grafu do ktorego punkt jest przypisany
            start_waiting_node = node[0]
            # wyznaczenie wezla poprzedzajacego
            end_waiting_node = [edge[0] for edge in self.graph.edges(data=True) if edge[1] == start_waiting_node][0]
            connected_poi = [edge[1] for edge in self.graph.edges(data=True) if edge[0] == start_waiting_node][0]
            graph_poi_node = self.graph.nodes[connected_poi]["sourceNode"]
            self.graph.edges[end_waiting_node, start_waiting_node]["connectedPoi"] = self.source_nodes[graph_poi_node][
                "poiId"]

        # pobranie id wezlow bazowych waiting-departure
        poi_wait_dep_nodes = [i for i in self.source_nodes if
                              self.source_nodes[i]["type"] == base_node_type["waiting-departure"]]
        for node_id in poi_wait_dep_nodes:
            # print("startWaitingNode", startWaitingNode)
            graph_node_ids = [node[0] for node in self.graph.nodes(data=True)
                              if node[1]["sourceNode"] == node_id
                              and node[1]["nodeType"] == new_node_type["intersection_in"]]
            # powinny byc dwa wezly, jeden laczy sie ze stanowiskiem, a drugi z wezlem
            # skrzyzowania i wchodzi w sklad krawedzi na ktorej kolejkują się roboty
            edges = [edge for edge in self.graph.edges(data=True)
                     if edge[1] in graph_node_ids]
            # powinny zostac znalezione dwie krawedzie spelniajace wymagania
            # rint("edges", edges)
            assert len(edges) == 2, "waiting path, departure-waiting poi error"
            if edges[0][2]["edgeGroupId"] != 0:
                # pierwsza zapisana krawedz zwiazana jest ze stanowiskiem
                # druga dotyczy oczekiwania
                start_waiting_node = edges[1][1]
                end_waiting_node = edges[1][0]
                source_id = self.graph.nodes[edges[0][0]]["sourceNode"]
            else:
                # druga zapisana krawedz dotyczy stanowiska, a pierwsza oczekiwania
                start_waiting_node = [0][1]
                end_waiting_node = edges[0][0]
                source_id = self.graph.nodes[edges[1][0]]["sourceNode"]
            self.graph.edges[end_waiting_node, start_waiting_node]["connectedPoi"] = \
                self.source_nodes[source_id]["poiId"]

    def add_nodes_position(self, combined_edges):
        for node_id, node_data in self.graph.nodes(data=True):
            node_position = self.source_nodes[node_data["sourceNode"]]["pos"]
            node_type = node_data["nodeType"]
            if node_type == new_node_type["dock"]:
                self.graph.nodes[node_id]["pos"] = self.get_poi_nodes_pos(node_id, combined_edges)
            elif node_type == new_node_type["wait"]:
                self.graph.nodes[node_id]["pos"] = self.get_poi_nodes_pos(node_id, combined_edges)
            elif node_type == new_node_type["undock"]:
                self.graph.nodes[node_id]["pos"] = self.get_poi_nodes_pos(node_id, combined_edges)
            elif node_type == new_node_type["end"]:
                self.graph.nodes[node_id]["pos"] = self.get_poi_nodes_pos(node_id, combined_edges)
            elif node_type == new_node_type["noChanges"]:
                self.graph.nodes[node_id]["pos"] = node_position
            elif node_type == new_node_type["intersection_in"]:
                position = self.get_new_intersection_node_position(node_id)
                self.graph.nodes[node_id]["pos"] = position
            elif node_type == new_node_type["intersection_out"]:
                position = self.get_new_intersection_node_position(node_id)
                self.graph.nodes[node_id]["pos"] = position

    def get_poi_nodes_pos(self, graph_node_id, combined_edges):
        # poszukiwanie id wezlow przed i za stanowiskiem na podstawie krawedzi grafu
        source_node_poi = self.graph.nodes[graph_node_id]
        source_node_poi_id = source_node_poi["sourceNode"]

        a = [data["sourceNodes"][0] for data in combined_edges.values()
             if data["sourceNodes"][-1] == source_node_poi_id]

        node_before_poi_id = a[0]
        b = [data["sourceNodes"][-1] for data in combined_edges.values()
             if data["sourceNodes"][0] == source_node_poi_id]

        node_after_poi_id = b[0]
        node_before_poi_pos = self.source_nodes[node_before_poi_id]["pos"]
        node_after_poi_pos = self.source_nodes[node_after_poi_id]["pos"]
        pA = [node_before_poi_pos[0], node_before_poi_pos[1]]
        pB = [node_after_poi_pos[0], node_after_poi_pos[1]]

        base_angle = math.radians(np.rad2deg(np.arctan2(pB[1] - pA[1], pB[0] - pA[0])))
        distance = math.sqrt(math.pow(pA[0] - pB[0], 2) + math.pow(pA[1] - pB[1], 2))
        translate_to_base_node = np.array([[1, 0, 0, node_before_poi_pos[0]],
                                          [0, 1, 0, node_before_poi_pos[1]],
                                          [0, 0, 1, 0],
                                          [0, 0, 0, 1]])

        rotation_to_way = np.array([[math.cos(base_angle), -math.sin(base_angle), 0, 0],
                                   [math.sin(base_angle), math.cos(base_angle), 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])

        step = distance / 5
        nodes_vect = []
        poi_step_translation = np.array([[1, 0, 0, step * source_node_poi["nodeType"]],
                                        [0, 1, 0, 0],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])
        node_translation = np.dot(np.dot(translate_to_base_node, rotation_to_way), poi_step_translation)
        if self.source_nodes[node_before_poi_id]["type"] == base_node_type["waiting-departure"]:
            s_id = self.graph.nodes[graph_node_id]["sourceNode"]
            nodes_vect.append([self.source_nodes[s_id]["pos"][0], self.source_nodes[s_id]["pos"][1]])
        else:
            nodes_vect.append([node_translation[0][3], node_translation[1][3]])

        return nodes_vect[0][0], nodes_vect[0][1]

    def get_new_intersection_node_position(self, graph_node_id):
        # A - punkt startowy krawedzi
        # B - punkt koncowy krawedzi
        # print("------------------------------")
        node_type = self.graph.nodes[graph_node_id]["nodeType"]
        source_node_id = self.graph.nodes[graph_node_id]["sourceNode"]
        main_graph_nodes_id = []
        for i in self.graph.edges:
            if i[0] == graph_node_id:
                main_graph_nodes_id.append(i[1])
            elif i[1] == graph_node_id:
                main_graph_nodes_id.append(i[0])

        end_graph_node_id = 0
        end_source_node_id = None
        for i in main_graph_nodes_id:
            if self.graph.nodes[i]["sourceNode"] != source_node_id:
                end_source_node_id = self.graph.nodes[i]["sourceNode"]
                end_graph_node_id = i
        pA = [self.source_nodes[source_node_id]["pos"][0], self.source_nodes[source_node_id]["pos"][1]]

        path_type = [edgeType[2]["wayType"] for edgeType in self.graph.edges(data=True)
                     if (edgeType[0] == graph_node_id and edgeType[1] == end_graph_node_id) or
                     (edgeType[1] == graph_node_id and edgeType[0] == end_graph_node_id)]
        if end_source_node_id is not None:
            graph_source_nodes = [edge[2]["sourceNodes"] for edge in self.graph.edges(data=True)
                                  if (edge[0] == graph_node_id and edge[1] == end_graph_node_id)
                                  or (edge[0] == end_graph_node_id and edge[1] == graph_node_id)]
            if len(graph_source_nodes[0]) >= 2:
                if graph_source_nodes[0][0] == source_node_id:
                    position = self.source_nodes[graph_source_nodes[0][1]]["pos"]
                else:
                    position = self.source_nodes[graph_source_nodes[0][-2]]["pos"]
            else:
                position = self.source_nodes[end_source_node_id]["pos"]
            pB = [position[0], position[1]]
            base_angle = math.radians(np.rad2deg(np.arctan2(pB[1] - pA[1], pB[0] - pA[0])))
            translate_to_base_node = np.array([[1, 0, 0, pA[0]],
                                              [0, 1, 0, pA[1]],
                                              [0, 0, 1, 0],
                                              [0, 0, 0, 1]])

            rotation_to_way = np.array([[math.cos(base_angle), -math.sin(base_angle), 0, 0],
                                       [math.sin(base_angle), math.cos(base_angle), 0, 0],
                                       [0, 0, 1, 0],
                                       [0, 0, 0, 1]])

            way_node = np.array([[1, 0, 0, corridor_width + robot_length],
                                [0, 1, 0, corridor_width / 2],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

            if node_type == new_node_type["intersection_out"]:
                way_node[1][3] = -corridor_width / 2

            if len(path_type) != 0:
                if path_type[0] != way_type["twoWay"]:
                    way_node[1][3] = 0
            way = np.dot(np.dot(translate_to_base_node, rotation_to_way), way_node)
            return way[0][3], way[1][3]
        else:
            return 0, 0

    def set_default_time_weight(self):
        robot_velocity = 0.5  # [m/s]
        for i in self.graph.edges:
            edge = self.graph.edges[i]
            is_blocked = False
            for e_id in edge["sourceEdges"]:
                if e_id != 0:
                    if not self.source_edges[e_id]["isActive"]:
                        is_blocked = True
                        break
            if is_blocked:
                self.graph.edges[i]["weight"] = None
            elif edge["behaviour"] == Task.beh_type["goto"] and edge["edgeGroupId"] != 0 \
                    and len(edge["sourceNodes"]) == 1:
                self.graph.edges[i]["weight"] = 3
            elif edge["behaviour"] == Task.beh_type["goto"]:
                nodes_pos = []
                for node_id in edge["sourceNodes"]:
                    nodes_pos.append(self.source_nodes[node_id]["pos"])
                dist = 0
                for j in range(len(nodes_pos) - 1):
                    dist = dist + math.hypot(nodes_pos[j + 1][0] - nodes_pos[j][0],
                                             nodes_pos[j + 1][1] - nodes_pos[j][1])
                self.graph.edges[i]["weight"] = math.ceil(dist / robot_velocity)
            elif edge["behaviour"] == Task.beh_type["dock"]:
                self.graph.edges[i]["weight"] = docking_time_weight
            elif edge["behaviour"] == Task.beh_type["wait"]:
                self.graph.edges[i]["weight"] = wait_weight
            elif edge["behaviour"] == Task.beh_type["undock"]:
                self.graph.edges[i]["weight"] = undocking_time_weight

    def set_max_robots(self):
        operation_pois = [i for i in self.source_nodes
                          if self.source_nodes[i]["type"]["nodeSection"]
                          in [base_node_section_type["dockWaitUndock"], base_node_section_type["waitPOI"]]
                          or self.source_nodes[i]["type"] == base_node_type["parking"]]
        for i in self.graph.edges:
            edge = self.graph.edges[i]
            no_poi_nodes = not (edge["sourceNodes"][0] in operation_pois or edge["sourceNodes"][-1] in operation_pois)
            if edge["behaviour"] == Task.beh_type["goto"] and len(edge["sourceNodes"]) != 1 and no_poi_nodes:
                nodes_pos = []
                for node_id in edge["sourceNodes"]:
                    nodes_pos.append(self.source_nodes[node_id]["pos"])
                dist = 0
                for j in range(len(nodes_pos) - 1):
                    dist = dist + math.hypot(nodes_pos[j + 1][0] - nodes_pos[j][0],
                                             nodes_pos[j + 1][1] - nodes_pos[j][1])
                self.graph.edges[i]["maxRobots"] = math.floor(dist / robot_length)

    def get_corridor_path(self, edge_id, source_path):
        # sourcePath = [(x,y),(x2,y2),...]
        line = LineString(source_path)
        corridor = line.buffer(corridor_width / 2, cap_style=3, join_style=2)
        unia = unary_union([corridor])
        x, y = unia.exterior.coords.xy
        source_path_corridor = [(x[i], y[i]) for i in range(len(x))]
        start_node_pos = [self.graph.nodes[edge[0]]["pos"] for edge in self.graph.edges(data=True)
                          if edge[2]["id"] == edge_id][0]
        end_node_pos = [self.graph.nodes[edge[1]]["pos"] for edge in self.graph.edges(data=True)
                        if edge[2]["id"] == edge_id][0]
        item_to_del = int((len(source_path_corridor) - 3) / 2)
        del source_path_corridor[0:item_to_del]
        del source_path_corridor[-3:]
        source_path_corridor = source_path_corridor[::-1]
        source_path_corridor[0] = start_node_pos
        source_path_corridor[-1] = end_node_pos
        return source_path_corridor

    def get_corridor_coordinates(self, edge_id):
        # zadania musza byc typu go to aby mozna bylo skorzystac
        edge = [data for data in self.graph.edges(data=True) if data[2]["id"] == edge_id][0]
        # print(edge)
        # print(len(edge["sourceNodes"]))
        assert Task.beh_type["goto"] == edge[2][
            "behaviour"], "Corridors can be only created to 'goto' behaviur edge type"
        # obsluga dla skrzyzowan
        if len(edge[2]["sourceNodes"]) == 1:
            #  print("skrzyzowanie")
            source_pos = self.source_nodes[edge[2]["sourceNodes"][0]]["pos"]
            # wezel startowy
            start_node_pos = [self.graph.nodes[data[0]]["pos"] for data in self.graph.edges(data=True)
                              if data[2]["id"] == edge_id][0]
            angle = np.arctan2(corridor_width + robot_length, corridor_width / 2) + np.arctan2(
                source_pos[1] - start_node_pos[1], source_pos[0] - start_node_pos[0])
            translate_to_base_node = np.array([[1, 0, 0, start_node_pos[0]], [0, 1, 0, start_node_pos[1]],
                                              [0, 0, 1, 0], [0, 0, 0, 1]])
            rotation_start_node = np.array([[math.cos(angle), -math.sin(angle), 0, 0],
                                           [math.sin(angle), math.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            corridor_start = np.array([[1, 0, 0, corridor_width / 2],
                                      [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            way = np.dot(np.dot(translate_to_base_node, rotation_start_node), corridor_start)
            start_pos = (way[0][3], way[1][3])

            # wezel koncowy
            end_node_pos = [self.graph.nodes[data[1]]["pos"] for data in self.graph.edges(data=True)
                            if data[2]["id"] == edge_id][0]
            angle2 = np.arctan2(corridor_width / 2, corridor_width + robot_length) + np.arctan2(
                end_node_pos[1] - source_pos[1], end_node_pos[0] - source_pos[0])
            translate_to_intersection = np.array([[1, 0, 0, source_pos[0]], [0, 1, 0, source_pos[1]],
                                                 [0, 0, 1, 0], [0, 0, 0, 1]])
            rotation_to_way2 = np.array([[math.cos(angle2), -math.sin(angle2), 0, 0],
                                        [math.sin(angle2), math.cos(angle2), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            way_node2 = np.array([[1, 0, 0, corridor_width + robot_length],
                                 [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            way2 = np.dot(np.dot(translate_to_intersection, rotation_to_way2), way_node2)
            end_pos = (way2[0][3], way2[1][3])

            #  startDifferentEdge = [data for data in self.graph.edges(data=True)\
            #      if data[1] == edge[0] and data[2]["wayType"] != edgeDirectionType["twoWay"]]
            #  if len(startDifferentEdge) > 0:
            #      startPos = startNodePos

            #  endDifferentEdge = [data for data in self.graph.edges(data=True)\
            #       if data[0] == edge[1] and data[2]["wayType"] != edgeDirectionType["twoWay"]]
            #   if len(endDifferentEdge) > 0:
            #      endPos = endNodePos
            #  print("start len", startDifferentEdge, "end len", endDifferentEdge)
            line = LineString([start_pos, source_pos, end_pos])
            corridor = line.buffer(corridor_width / 2, cap_style=3, join_style=2)
            unia = unary_union([corridor])
            x, y = unia.exterior.coords.xy
            base_coordinates = [(x[i], y[i]) for i in range(len(x))]

            x_source = [start_node_pos[0], source_pos[0], end_node_pos[0]]
            y_source = [start_node_pos[1], source_pos[1], end_node_pos[1]]
            x_path = [point[0] for point in [start_pos, source_pos, end_pos]]
            y_path = [point[1] for point in [start_pos, source_pos, end_pos]]
            x_cor = [point[0] for point in base_coordinates]
            y_cor = [point[1] for point in base_coordinates]
            plt.plot(x_source, y_source, "g", x_path, y_path, "r", x_cor, y_cor, "b")
            # plt.plot(x_source,y_source,"g",x_path,y_path,"r")
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.show()
            return base_coordinates
        else:
            print("inne krawedzie")
            # inne krawedzie
            source_id = [data[2]["sourceNodes"] for data in self.graph.edges(data=True)
                         if data[2]["id"] == edge_id][0]
            source_path = [self.source_nodes[i]["pos"] for i in source_id]
            path_coordinates = self.get_corridor_path(edge_id, source_path)
            return create_corridor_coordinates(path_coordinates)

    def print_corridor(self, edge_id):
        # pobranie wspolrzednych wezlow z grafu wlasciwego
        source_id = [data[2]["sourceNodes"] for data in self.graph.edges(data=True) if data[2]["id"] == edge_id][0]
        source_path = [self.source_nodes[i]["pos"] for i in source_id]
        # przesuniecie i wygenerowanie lamanej po ktorej bedzie poruszal sie robot
        path_coordinates = self.get_corridor_path(edge_id, source_path)
        # wygenerowanie korytarza
        corridor_coordinates = self.get_corridor_coordinates(edge_id)
        # wyswietlenie korytarza
        x_source = [point[0] for point in source_path]
        y_source = [point[1] for point in source_path]
        x_path = [point[0] for point in path_coordinates]
        y_path = [point[1] for point in path_coordinates]
        x_cor = [point[0] for point in corridor_coordinates]
        y_cor = [point[1] for point in corridor_coordinates]
        plt.plot(x_source, y_source, "g", x_path, y_path, "r", x_cor, y_cor, "b")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.show()

    def get_graph(self):
        return self.graph

    def print_graph(self, plot_size=(45, 45)):
        plt.figure(figsize=plot_size)
        node_pos = nx.get_node_attributes(self.graph, "pos")

        max_robots = nx.get_edge_attributes(self.graph, "maxRobots")
        # edge_col = [G[u][v]["color"] for u,v in self.graph.edges()]

        node_col = [self.graph.nodes[i]["color"] for i in self.graph.nodes()]
        # nx.draw_networkx(self.graph, node_pos,node_color = node_col, node_size=550,font_size=15,
        # with_labels=True,font_color="w", width=2)

        nx.draw_networkx(self.graph, node_pos, node_color=node_col, node_size=3000, font_size=25,
                         with_labels=True, font_color="w", width=4)
        nx.draw_networkx_edge_labels(self.graph, node_pos, node_color=node_col,
                                     edge_labels=max_robots, font_size=30)

        # nx.draw_networkx(self.graph, node_pos,edge_color= edge_col, node_color = node_col, node_size=3000,
        # font_size=25,with_labels=True,font_color="w", width=4)
        # nx.draw_networkx_edge_labels(self.graph, node_pos, edge_color= edge_col, node_color = node_col,
        # edge_labels=maxRobots,font_size=30)
        plt.show()
        plt.close()
