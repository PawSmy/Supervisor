class Task():
    """
    Attributes:
        priority (dictionary): slownik wartosci priorytetow
        behType (dictionary): slownik wartosci zachowan dla robota
        status (dictionary): slownik wartosci statusow zadania
    """
    priority = {
        "low": 1, #zadanie malo istotne
        "normal": 2, #normalne zadania generowane przez system
        "high": 3, #dla zadan wymagajacych zjazdu do stacji ladowania
        "veryHigh": 4 #akcje zlecone przez serwisanta           
    }
    
    behType = {
        "goto": 1,
        "dock": 2,
        "wait": 3,
        "undock": 4
    }   
    
    status = {
        "inProgress" : 1,
        "done": 2
    }
    
    def __init__(self, data):
        """
        Attributes:
            data ({"id": int, "behaviours": [{"id": int, "status": Task.status, "goalId": 
                                            int, "name":Task.behType["..."]},...],
                  "robotId": int, "timeAdded": time, "priority": Task.priority["..."]}): zadanie dla robota
        """
        self.data = data
    
    def getFirstUndoneBehaviour(self):
        """
        Jesli dla zadania istnieja niewykonane zachowania to zwracane jest niewykonane zachowanie, jesli 
        takiego nie ma to zwracana jest wartosc None.
        Returns:
            behaviour ({"id": int, "status": Task.status, "goalId": int, "name":Task.behType["..."]}):
                pierwsze niewykonane zadanie, jesli istnieje w przeciwnym wypadku None
        """
        for behaviour in self.data["behaviours"]:
            if behaviour["status"] != self.status["done"]:
                return behaviour
        return None
    
    def getPoiGoal(self):
        goalPoi = None
        previousBehaviour = self.data["behaviours"][0]
        for behaviour in self.data["behaviours"]:
            if "goalId" in behaviour:
                if goalPoi == None or behaviour["status"] == self.status["done"]:
                    #własciwy dojazd do POI został wykonany, ale kolejnym zachowaniem może być
                    #dokowanie,wait,oddokowanie
                    goalPoi = behaviour["goalId"]
                if previousBehaviour["status"] == "done" and behaviour["status"] != self.status["done"]:
                    #poprzednie zachowanie zostało zakończone, aktualne posiada jakiś inny status
                    #dla tego nowego zachowania robot będzie jechał do POI przez nie wskazanego
                    goalPoi = behaviour["goalId"]
            previousBehaviour = behaviour
        return goalPoi

    def getCurrentBehaviour(self):
        for behaviour in self.data["behaviours"]:
            if behaviour["status"] != self.status["done"]:
                return behaviour
            
    def getTaskFirstGoal(self):
        """Dla zadania zwracane jest pierwsze POI do ktorego ma dojechac robot w ramach podanego zadania.

        Parameters:
            task ({"id": int, "behaviours": BehavioursList, "robotId": int, "timeAdded": dataTime, "priority": int}):
                 zadanie dla ktorego ma zostac okreslone id POI z bazy do ktorego ma dojecha robot

        Returns:
            goalId (int): id POI z bazy
        """
        for behaviour in self.data["behaviours"]:
             if "goalId" in behaviour:
                return behaviour["goalId"]   
