import json
import requests
import time
import graph_creator as gc

class graphApi:
  updateDelay = 60
  graphList = {}

  def __init__(self):
    self.updateAll()

  def updateAll(self):
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

    # Tutaj do ustalenia jak generować listy z formatu zapisu b. danych
    graf = gc.SupervisorGraphCreator(source_nodes, source_edges, pois_raw_data)

    self.graphList[gID] = { "ts": time.time(), "edges": gData, "graf": graf }

  def getGraphByID(self, gID):
    self.updateOne(gID)
    return self.graphList[gID]["graf"]

  @property
  def graphIDlist(self):
    return self.graphList.keys()

# All below is for selfstanfing class, this is part of supervisor.py
class apiEndpoint:
  endpoints = {
    "login":"http://helike-ra-back-sosnus-develop.azurewebsites.net/users/login",
    "getGraphs":"http://helike-ra-back-sosnus-develop.azurewebsites.net/graphs/all",
    "getGraph":"http://helike-ra-back-sosnus-develop.azurewebsites.net/graphs/"
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