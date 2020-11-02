import json
import requests
import threading

class graphApi:

  graphUpdateCheckRate = 5.0
  currentGraph = None
  lastTimestamp = None


  def __init__(self):
    self.checkLastUpdate()

  def processGraphData(self, rawData):
    #graph processing
    processedData = rawData
    #end graph processing
    self.currentGraph = processedData

  def updateGraphData(self):
    response = self.getGraphData
    if(response.status_code == 200):
      respdata = response.json()
      if not respdata.data == None:
        self.processGraphData(respdata)

  def checkLastUpdate(self):
    response = self.getGraphTimestamp
    if(response.status_code == 200):
      newts = response.json()
      if not newts.timestamp == self.lastTimestamp:
        self.lastTimestamp = newts.timestamp
        self.updateGraphData()

    threading.Timer(self.graphUpdateCheckRate,self.checkLastUpdate).start()



class apiEndpoint:
  endpoints = {
    "login":"http://tebe.westeurope.cloudapp.azure.com:3333/users/login",
    "getGraph":None,
    "getGraphTimestamp":None
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

  def getGraphTimestamp(self):
    return self._raptorsAPI.get(self.endpoints["getGraphTimestamp"])

  def getGraphData(self):
    return self._raptorsAPI.get(self.endpoints["getGraph"])

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