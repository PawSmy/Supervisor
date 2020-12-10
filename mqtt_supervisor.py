import json
import requests
import threading
import paho.mqtt.client as mqtt
import graphProcessing as gp

#global variables
robotsDict = {}
robotsVerifyQueue = {}
operationPointsDict = {}
DEBUG = 1

#api connection class
class apiEndpoint:
  robotsVerifyCheckRate = 15.0
  apiUpdateRate = 2.0
  backServer = "http://helike-ra-back-sosnus-develop.azurewebsites.net/"
  endpoints = {
    "login":"users/login",
    "updateRobo":"robots/update",
    "getRobotsID":"robots/allById",
    "getRobotsByID":"robots/",
    "checkRobotsByID":None,
    "getGraphs":"graphs/all",
    "getGraph":"graphs/",
    "getStands":"movement/stands/all"
  }

  def __init__(self,login,password):
    self._login = login
    self._pass = password
    self._raptorsAPI = requests.Session()
    self._raptorsAPI.auth = (login,password)

    # login check
    self.apiLoginRESP = self._raptorsAPI.post(self.backServer + self.endpoints["login"], json = {"email":login,"password":password})

  def __del__(self):
    self._raptorsAPI.close()

  def connectionCheck(self):
    return (self.apiLoginRESP.status_code == 200)

  def updateRoboData(self, jsonData):
    return self._raptorsAPI.post(self.backServer + self.endpoints["updateRobo"], json = jsonData)

  def apiUpdateLoop(self):
    for r in robotsDict.values():
      if(r.odomData != "" and r.odomData != r.lastOdomData):
        # clear heartbeat counter
        r.hbClear()
        # move current data to buffer
        r.lastOdomData = r.odomData
        odomPos = r.odomData.split("|")
        json = {"id":r.apiID,"pose":{"position":{"x": odomPos[0],"y": odomPos[1],"z": odomPos[2]}}}
        apiRESP = self.updateRoboData(json)
        if DEBUG > 0:
          print("Status:{} Dane:{}".format(apiRESP.status_code,json))
    threading.Timer(self.apiUpdateRate,self.apiUpdateLoop).start()

  def getAllRobotsID(self):
    return self._raptorsAPI.get(self.backServer + self.endpoints["getRobotsID"])
  def getSingleRoboByID(self,roboID):
    return self._raptorsAPI.get(self.backServer + self.endpoints["getRobotsByID"]+str(roboID))

  def verifyRobots(self):
    if len(robotsVerifyQueue)>1:
      response = self.getAllRobotsID()
      if(response.status_code == 200):
        for id in response.json():
          if id != None and id in robotsVerifyQueue:
            robotsDict[id] = robotsVerifyQueue[id]
            del robotsVerifyQueue[id]
    else:
      id = next(iter(robotsVerifyQueue.keys()))
      response = self.getSingleRoboByID(id)
      if(response.status_code == 200):
        robotsDict[id] = robotsVerifyQueue[id]
        del robotsVerifyQueue[id]

  def verifyRobotsLoop(self):
    if robotsVerifyQueue:
      #self.verifyRoboQueue()
      self.verifyRobots()
    threading.Timer(self.robotsVerifyCheckRate,self.verifyRobotsLoop).start()

  def getOneGraph(self,gID):
    return self._raptorsAPI.get(self.endpoints["getGraph"]+str(gID))

  def getAllGraphs(self):
    return self._raptorsAPI.get(self.endpoints["getGraphs"])

  def getAllStands(self):
    return self._raptorsAPI.get(self.endpoints["getStands"])

# =====================================================================================================================
  # dla nowego endpointu
  def checkByRoboIdList(self,IDlist):
    # IDlist = [id1,id2,id3]
    # nie ma jeszcze endopinta dodanego do dict tutaj (bo nie wiem jaki ma adres, jak go zrobimy trzeba bedzie dopisac)
    return self._raptorsAPI.get(self.backServer + self.endpoints["checkRobotsByID"], json = IDlist)

  def verifyRoboQueue(self):
    # wersja dla odpowiedzi w formacie: {id1:0/1,id2:0/1} ~ chyba najlepsza wersja bo i tak kolejka to dict.
    if robotsVerifyQueue:
      response = self.checkByRoboIdList(robotsVerifyQueue.keys())
      if(response.status_code == 200):
        jsonChecklist = response.json()
        for id in robotsVerifyQueue.keys():
          if id != None and id in jsonChecklist.keys():
            if jsonChecklist[id]:
              robotsDict[id] = robotsVerifyQueue[id]
              del robotsVerifyQueue[id]
# =====================================================================================================================

#robo handling class
class roboHandler:
  
  heartbeatCNT = 0
  heartbeatMaxVal = 5
  odomData = ""
  lastOdomData = ""

  def __init__(self,roboApiId):
    self.apiID = roboApiId

  # heartbeat functions
  def hbClear(self):
    self.heartbeatCNT = 0

  def hbUpdate(self):
    self.heartbeatCNT += 1
    # alert
    if self.heartbeatCNT > self.heartbeatMaxVal:
      print("No new data from:{}".format(self.apiID))

  # data recieve
  def odomUpdate(self,data):
    self.odomData = data

#task points class
class operationPoints:
  
  currentRobo = ""

  def __init__(self,pointApiId):
    self.apiID = pointApiId

  def releaseRobo(self):
    self.currentRobo = ""
    return 1

  def assignRobo(self,roboID):
    if(self.currentRobo):
      return 0
    else:
      self.currentRobo = roboID
      return 1

#raw data handling class
class mqttHandler:

  def __init__(self,ip,port,alive):
    self._ip = ip
    self._port = port
    self._alive = alive
    self._client = mqtt.Client()
    self._client.connect(ip,port,alive)
    self._client.on_connect = self.mqtt_on_connect
    self._client.loop_start()

    # temp subscribe definition
    self._client.message_callback_add("topic/odomRawPOS", self.odomPos)
    self._client.subscribe("topic/odomRawPOS")
    self._client.message_callback_add("topic/assignRelease", self.assignReleaseFromPoint)
    self._client.subscribe("topic/assignRelease")

  def __del__(self):
    self._client.loop_stop()
    self._client.disconnect()

  #mqtt conection status
  def mqtt_on_connect(self, client, userdata, flags, rc):
    print("MQTT connected with result code "+str(rc))

  def odomPos(self, client, userdata, msg):
    rawOdom = msg.payload.decode()
    # split to id and data
    odomrData = rawOdom.split("||")
    
    if odomrData[0] in robotsDict:
      # update data in robo object
      robotsDict[odomrData[0]].odomUpdate(odomrData[1])
    elif odomrData[0] not in robotsVerifyQueue:
      # add new robo obiect to dict
      robotsVerifyQueue[odomrData[0]] = roboHandler(odomrData[0])
      rAPI.verifyRobots()
      #rAPI.verifyRoboQueue()

  # heartbeat loop
  def supervisorHB(self):
    self._client.publish("topic/supervisorHB","ping",1)
    for r in robotsDict.values():
      r.hbUpdate()
    threading.Timer(5.0,self.supervisorHB).start()

  # ===================================================================================================================
  def assignReleaseFromPoint(self, client, userdata, msg):
    # format danych dla przypisywania: 1||pointID|roboID
    # format danych dla zwalniania: 0||pointID
    rawdata = msg.payload.decode()
    optype = rawdata.split("||")
    idList = optype[1].split("|")
    if idList[0] in operationPointsDict:
      if(optype[0]):
        # przypisywanie
        if idList[1] in robotsDict:
          status = operationPointsDict[idList[0]].assignRobo(idList[1])
          if(not status):
            print ("id juz jest przypisane dla tego punktu (w tym miejscu mozna obsluzyc zdarzenie)")
      else:
        # zwalnianie
        operationPointsDict[idList[0]].releaseRobo()
  # ===================================================================================================================

# init program
rAPI = apiEndpoint('aaa','aaa')
if(rAPI.connectionCheck):
  print ("API connection success")

  #create graph class
  gAPI = gp.graphApi()

  #connect mqtt
  rMQTT = mqttHandler("localhost",1883,60)

  # init api update loop and heartbeat
  rAPI.apiUpdateLoop()
  rAPI.verifyRobotsLoop()
  rMQTT.supervisorHB()
  
  # program loop
  try:
    while True:
      pass
  except KeyboardInterrupt:
    del rAPI
    del rMQTT
else:
  print ("API connection failed: response code", rAPI.apiLoginRESP.status_code)