import json
import requests
import time
import graph_creator as gc

class graphApi:
  updateDelay = 60
  graphList = {}
  standsList = []
  def __init__(self):
    #self.updateAll()
    response = rAPI.getAllStands()
    if(response.status_code == 200):
      self.standsList = response.json()
      self.updateOne("5f996c6f65d0891beaa67bdf")

      #self.updateRoboOnEdge("5f996c6f65d0891beaa67bdf","robo1",(1,2))

  def updateAll(self):
    response = rAPI.getAllStands()
    if(response.status_code == 200):
      self.standsList = response.json()
      
      response = rAPI.getAllGraphs()
      if(response.status_code == 200):
        grafy = response.json()
        for graf in grafy:
          self.updateGraphInDict(graf["id"],graf["edges"])

  def updateOne(self,gID):
    if gID in self.graphList.keys():
      if time.time() - self.graphList[gID].ts < self.updateDelay:
        return 0

    response = rAPI.getOneGraph(gID)
    if(response.status_code == 200):
      graf = response.json()
      self.updateGraphInDict(graf["id"],graf["edges"])

  def updateGraphInDict(self,gID,gData):
    if gID in self.graphList.keys():
      if self.graphList[gID] == gData:
        return 0

    gProcessed = self.processGraf(gData)
    graf = gc.SupervisorGraphCreator(gProcessed["node_list"], gProcessed["edge_list"], self.standsList)
    self.rewriteRobotsOnEdges(gID,graf)
    self.graphList[gID] = { "ts": time.time(), "edges": gData, "graf": graf }


  def getGraphByID(self, gID, checkUpdate = False):
    if gID in self.graphList.keys():
      if checkUpdate: self.updateOne(gID)
      return self.graphList[gID]["graf"]
    return None

  @property
  def graphIDlist(self):
    return self.graphList.keys()

  def processGraf(self, gData):
    gProcessed = {"node_list":{},"edge_list":{}}

    # create gc.base_node_type map
    gcBNT = {}
    for key, val in gc.base_node_type.items():
      gcBNT[val["id"]] = key

    # graph processing
    nodeKey = 1
    edgeKey = 1
    for edge in gData:
      startNode = 0
      endNode = 0
      nodeNo = 0
      # node processing
      for node in edge["verticesList"]:
        if not node["type"] == None:
          pos = (node["posX"],node["posY"])
          # checking if node already added
          for eKey, data in gProcessed["node_list"].items():
            if data["pos"] == pos:
              if nodeNo == 0: startNode = eKey
              else: endNode = eKey
          
          # adding new node
          if nodeNo - startNode == 0 or nodeNo - endNode == 1:
            name = "N"+str(nodeKey)

            nType = gcBNT[14] # intersection
            poiId = 0         # none
            if node["type"] > 0: nType = gcBNT[node["type"]]
            else:
              poiId = node["poiID"]
              nType = gcBNT[1] # <======= na sztywno bo nie wiem jak to wyciągnąć z poi

            gProcessed["node_list"][nodeKey] = {"name": name, "pos": pos, "type": gc.base_node_type[nType], "poiId": poiId}

            if nodeNo == 0: startNode = nodeKey
            else: endNode = nodeKey

            nodeKey += 1

        nodeNo += 1

      # edge processing
      edgeType = "twoWay"
      if not edge["biDirected"]: edgeType = "oneWay"
      elif edge["narrow"]: edgeType = "narrowTwoWay"

      gProcessed["edge_list"][edgeKey] = {"startNode": startNode, "endNode": endNode, "type": gc.way_type[edgeType], "isActive": edge["isActive"]}
      edgeKey += 1
    
    return gProcessed
  
  def updateRoboOnEdge(self,gID,rID,edge): #graph ID, robo ID, edge vector (u,v)
    graf = self.getGraphByID(gID,checkUpdate = True).get_graph()
    self.removeRoboFromEdge(gID,rID)
    graf[edge[0]][edge[1]]["robotsList"].append(rID)
  
  def removeRoboFromEdge(self,gID,rID): #graph ID, robo ID
    graf = self.getGraphByID(gID,checkUpdate = False).get_graph()
    for edge in graf.edges(data=True):
      if "robotsList" in edge[2].keys():
        if rID in edge[2]["robotsList"]:
          edge[2]["robotsList"].remove(rID)

  def rewriteRobotsOnEdges(self,gID,dst):
    if gID in self.graphList.keys():
      src = self.getGraphByID(gID,checkUpdate = False).get_graph()
      for edge in dst.get_graph().edges(data=True):
        if src[edge[0]][edge[1]]:
          edge[2]["robotsList"] = src[edge[0]][edge[1]]["robotsList"]
        else:
          edge[2]["robotsList"] = []
    else:
      for edge in dst.get_graph().edges(data=True):
        edge[2]["robotsList"] = []

# All below is for selfstanfing class, this is part of supervisor.py
class apiEndpoint:
  endpoints = {
    "login":"http://helike-ra-back-sosnus-develop.azurewebsites.net/users/login",
    "getGraphs":"http://helike-ra-back-sosnus-develop.azurewebsites.net/graphs/all",
    "getGraph":"http://helike-ra-back-sosnus-develop.azurewebsites.net/graphs/",
    "getStands":"http://helike-ra-back-sosnus-develop.azurewebsites.net/movement/stands/all"
  }

  def __init__(self,login,password):
    self._login = login
    self._pass = password
    self._raptorsAPI = requests.Session()
    self._raptorsAPI.auth = (login,password)

    # login check
    self.apiLoginRESP = self._raptorsAPI.post(self.endpoints["login"], json = {"email":login,"password":password})

  def __del__(self):
    self._raptorsAPI.close()

  def connectionCheck(self):
    return (self.apiLoginRESP.status_code == 200)

  def getOneGraph(self,gID):
    return self._raptorsAPI.get(self.endpoints["getGraph"]+str(gID))

  def getAllGraphs(self):
    return self._raptorsAPI.get(self.endpoints["getGraphs"])

  def getAllStands(self):
    return self._raptorsAPI.get(self.endpoints["getStands"])

rAPI = apiEndpoint('aaa','aaa')
if(rAPI.connectionCheck):
  print ("API connection success")

  gAPI = graphApi()
  try:
    while True:
      pass
  except KeyboardInterrupt:
    del rAPI
else:
  print ("API connection failed: response code", rAPI.apiLoginRESP.status_code)