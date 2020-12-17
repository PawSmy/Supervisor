# -*- coding: utf-8 -*-
import copy
import networkx as nx
import graph_creator as gc
import numpy as np
import time
from datetime import datetime


class DispatcherError(Exception):
    """Base class for exceptions in this module."""
    pass


class TimeoutPlanning(DispatcherError):
    """Planning timeout passed."""
    pass


class WrongData(DispatcherError):
    """Wrong data"""


class WrongBehaviourInputData(WrongData):
    """Wrong behaviour input data"""


class WrongTaskInputData(WrongData):
    """Wrong task input data"""


class WrongRobotInputData(WrongData):
    """Wrong robot input data"""


class TaskManagerError(DispatcherError):
    """Task manager error"""


class PoisManagerError(DispatcherError):
    """Pois manager error"""


class PlaningGraphError(DispatcherError):
    """Planing graph error"""


class Behaviour:
    """
    Klasa zawierajaca informacje o pojedynczym zachowaniu dla robota

    Attributes:
        id (string): id zachowania dla robota w ramach zadania
        parameters (dict): slownik z parametrami dla robota, moze sie roznic w zaleznosci od typu zachowania
    """
    PARAM = {
        "ID": "id",  # nazwa id zachowania w bazie
        "BEH_PARAM": "parameters",  # nazwa pola zawierajacego liste parametrow
        "TYPE": "name",  # nazwa typu zachowania
        "BEH_POI": "to"  # nazwa pola, ktore odnosi sie do celu (POI)
    }
    TYPES = {  # slownik wartosci zachowan dla robota, wartoscia jest stala nazwa dla danego typu zachowania
        "goto": "GO_TO",
        "dock": "DOCK",
        "wait": "3",
        "bat_ex": "BAT_EX",
        "undock": "UNDOCK"
    }

    def __init__(self, behaviour_data):
        """
        Parameters:
            behaviour_data (dict): slownik z parametrami zachowania
                dict {"id" string: , "parameters": {"name": Behaviour.TYPES[nazwa_typu], "to": "id_poi"})
        """
        self.validate_data(behaviour_data)
        self.id = behaviour_data[self.PARAM["ID"]]
        self.parameters = behaviour_data[self.PARAM["BEH_PARAM"]]

    def __getitem__(self, key):
        return self.parameters[key]

    def __contains__(self, key):
        return key in self.parameters

    def get_type(self):
        """
        Returns:
            (string): typ zachowania
        """
        return self.parameters[self.PARAM["TYPE"]]

    def check_if_go_to(self):
        """
        Returns:
            (bool): informacja o tym czy zachowanie jest typu GO TO
        """
        return self.parameters[self.PARAM["TYPE"]] == self.TYPES["goto"]

    def get_poi(self):
        """
        Returns:
             (string): zawiera informacje z id POI dla zachowania GO TO, a dla pozostalych wartosc None
        """
        if self.check_if_go_to():
            return self.parameters[self.PARAM["BEH_POI"]]
        else:
            return None

    def validate_data(self, behaviour_data):
        beh_type = type(behaviour_data)
        if beh_type is not dict:
            raise WrongBehaviourInputData("Behaviour should be dict type but {} was given.".format(beh_type))
        base_beh_info_keys = list(behaviour_data.keys())
        if self.PARAM["ID"] not in base_beh_info_keys:
            raise WrongBehaviourInputData("Behaviour param '{}' name doesn't exist.".format(self.PARAM["ID"]))
        if self.PARAM["BEH_PARAM"] not in base_beh_info_keys:
            raise WrongBehaviourInputData("Behaviour param '{}' name doesn't exist.".format(self.PARAM["BEH_PARAM"]))

        beh_id_type = type(behaviour_data[self.PARAM["ID"]])
        if beh_id_type is not str:
            raise WrongBehaviourInputData("Behaviour ID should be string but '{}' was given".format(beh_id_type))
        beh_param_type = type(behaviour_data[self.PARAM["BEH_PARAM"]])
        if beh_param_type is not dict:
            raise WrongBehaviourInputData("Behaviour parameters should be dict but '{}' "
                                          "was given".format(beh_param_type))

        param_keys = list(behaviour_data[self.PARAM["BEH_PARAM"]].keys())
        if self.PARAM["TYPE"] not in param_keys:
            raise WrongBehaviourInputData("In behaviours parameters '{}' name doesn't "
                                          "exist.".format(self.PARAM["TYPE"]))
        beh_name = behaviour_data[self.PARAM["BEH_PARAM"]][self.PARAM["TYPE"]]
        if type(beh_name) is not str:
            raise WrongBehaviourInputData("Behaviour '{}' should be str.".format(type(beh_name)))
        if beh_name not in self.TYPES.values():
            raise WrongBehaviourInputData("Behaviour '{}' doesn't exist.".format(beh_name))

        if beh_name == self.TYPES["goto"]:
            if self.PARAM["BEH_POI"] not in param_keys:
                raise WrongBehaviourInputData("Missing '{}' name for {} behaviour.".format(self.PARAM["BEH_POI"],
                                                                                           self.TYPES["goto"]))
            beh_type = type(behaviour_data[self.PARAM["BEH_PARAM"]][self.PARAM["BEH_POI"]])
            if beh_type is not str:
                raise WrongBehaviourInputData("Behaviour goto poi id should be a string "
                                              "but {} was given.".format(beh_type))

    def get_info(self):
        """
        Wyswietla informacje o zachownaiu
        """
        return "id: " + str(self.id) + ", parameters: " + str(self.parameters) + "\n"


class Task:
    """
    Klasa przechowuje informacje o pojedymczym zadaniu w systemie

    Attributes:
        id (string): id zadania
        robot_id (string): id robota
        start_time (time_string YYYY-mm-dd HH:MM:SS or None): czas dodania zadania do bazy
        behaviours ([Behaviour, Behaviour, ...]): lista kolejnych zachowan dla robota
        curr_behaviour_id (int): id aktualnie wykonywanego zachowania, jesli zadanie jest w trakcie wykonywania
        status (string): nazwa statusu z listy STATUS_LIST
        weight (float): waga z jaka powinno zostac wykonane zadanie, im wyzsza tym wyzszy priorytet
    """
    PARAM = {
        "ID": "id",  # nazwa pola zawierajacego id zadania
        "ROBOT_ID": "robot",  # nazwa pola zawierajacego id robota
        "START_TIME": "start_time",  # nazwa pola zawierajaca czas pojawienia sie zadania w systemie
        "CURRENT_BEH_ID": "current_behaviour_index",  # nazwa pola odnoszacego sie do aktualnie wykonywanego
        # zachowania po id
        "STATUS": "status",  # nazwa pola zawierajacego status zadania
        "WEIGHT": "weight",  # nazwa pola odnoszacego sie do wagi zadania
        "BEHAVIOURS": "behaviours",  # nazwa pola odnoszaca sie do listy zachowan w zadaniu
        "PRIORITY": "priority"  # priorytet zadania
    }

    STATUS_LIST = {  # slownik wartosci nazw statusow zadania
        "TO_DO": 'To Do',  # nowe zadanie, nie przypisane do robota
        "IN_PROGRESS": "IN_PROGRESS",  # zadanie w trakcie wykonywania
        "ASSIGN": "ASSIGN",  # zadanie przypisane, ale nie wykonywane. Oczekiwanie na potwierdzenie od robota
        "DONE": "COMPLETED"  # zadanie zakonczone
    }

    def __init__(self, task_data):
        """
        Attributes:
            task_data ({"id": string, "behaviours": [Behaviour, Behaviour, ...],
                  "robotId": string, "start_time": time, "priority": int, "status": STATUS_LIST[status},
                  "weight": int): zadanie dla robota
        """
        # self.validate_input(task_data)
        self.id = task_data[self.PARAM["ID"]]
        self.robot_id = task_data[self.PARAM["ROBOT_ID"]]
        self.start_time = task_data[self.PARAM["START_TIME"]]
        self.status = task_data[self.PARAM["STATUS"]]
        self.weight = task_data[self.PARAM["WEIGHT"]]
        self.priority = 3 if self.PARAM["PRIORITY"] not in task_data else task_data[self.PARAM["PRIORITY"]]
        self.index = 0
        self.curr_behaviour_id = task_data[self.PARAM["CURRENT_BEH_ID"]]  # dla statusu done kolejne zachowania
        # jesli zadanie ma inny status to wartosc tyczy sie aktualnie wykonywanego zachowania
        try:
            self.behaviours = [Behaviour(raw_behaviour) for raw_behaviour in task_data[self.PARAM["BEHAVIOURS"]]]
        except WrongBehaviourInputData as error:
            raise WrongTaskInputData("Task id: {}. Behaviour error. {}".format(task_data[self.PARAM["ID"]], error))

    def get_poi_goal(self):
        """
        Returns:
            (string): id POI w kierunku, ktorego porusza sie robot lub id POI, w ktorym wykonuje zachowanie
        """
        current_behaviour_to_do = self.get_current_behaviour()
        goal_poi = None
        previous_behaviour = None

        for behaviour in self.behaviours:
            if current_behaviour_to_do.id == behaviour.id:
                if behaviour.check_if_go_to():
                    goal_poi = behaviour.get_poi()
                else:
                    if previous_behaviour is not None:
                        goal_poi = previous_behaviour.get_poi()
                        break

            if behaviour.check_if_go_to():
                previous_behaviour = behaviour

        return goal_poi if goal_poi is not None else previous_behaviour.get_poi()

    def get_current_behaviour(self):
        """
        Zwraca aktualne zachowanie wykonywane przez robota lub takie ktore ma byc kolejno wykonane, bo poprzednie
        zostalo zakonczone.

        Returns:
            (Behaviour): aktualnie wykonywane zachowanie w ramach zadania
        """
        return self.behaviours[0] if self.curr_behaviour_id == -1 else self.behaviours[self.curr_behaviour_id]

    def check_if_task_started(self):
        """
        Sprawdza czy dane zadanie zostalo rozpoczete.

        Returns:
            (bool): wartosc True jesli zadanie zostalo rozpoczete w przeciwnym wypadku False
        """
        return self.status != self.STATUS_LIST["TO_DO"]

    def validate_input(self, task_data):
        """
        Attributes:
            task_data ({"id": string, "behaviours": [Behaviour, Behaviour, ...],
                  "robotId": string, "timeAdded": time, "PRIORITY": Task.PRIORITY["..."]}): zadanie dla robota
        """
        if type(task_data) != dict:
            raise WrongTaskInputData("Wrong task input data type.")
        task_keys = task_data.keys()
        # sprawdzenie czy zadanie zawiera wszystkie niezbedne parametry
        if self.PARAM["ID"] not in task_keys:
            raise WrongTaskInputData("Task param '{}' doesn't exist.".format(self.PARAM["ID"]))
        task_id = task_data[self.PARAM["ID"]]
        if self.PARAM["ROBOT_ID"] not in task_keys:
            raise WrongTaskInputData("Task id: {}. Param '{}' doesn't exist.".format(task_id, self.PARAM["ROBOT_ID"]))
        if self.PARAM["START_TIME"] not in task_keys:
            raise WrongTaskInputData("Task id: {}. Param '{}' doesn't exist.".format(task_id, self.PARAM["START_TIME"]))
        if self.PARAM["BEHAVIOURS"] not in task_keys:
            raise WrongTaskInputData("Task id: {}. Param '{}' doesn't exist.".format(task_id, self.PARAM["BEHAVIOURS"]))
        if self.PARAM["CURRENT_BEH_ID"] not in task_keys:
            raise WrongTaskInputData("Task id: {}. Param '{}' doesn't exist.".format(task_id,
                                                                                     self.PARAM["CURRENT_BEH_ID"]))
        if self.PARAM["STATUS"] not in task_keys:
            raise WrongTaskInputData("Task id: {}. Param '{}' doesn't exist.".format(task_id, self.PARAM["STATUS"]))
        if self.PARAM["WEIGHT"] not in task_keys:
            raise WrongTaskInputData("Task id: {}. Param '{}' doesn't exist.".format(task_id, self.PARAM["WEIGHT"]))

        # sprawdzenie czy parametry sa wlasciwego typu
        task_id_type = type(task_id)
        if task_id_type is not str:
            raise WrongTaskInputData("Task '{}' should be str type but {} was given.".format(self.PARAM["ID"],
                                                                                             task_id_type))

        robot_id = task_data[self.PARAM["ROBOT_ID"]]
        task_robot_type = type(robot_id)
        if task_robot_type != str and robot_id is not None:
            raise WrongTaskInputData("Task id: {}. Param '{}' should be str or None type but {} "
                                     "was given.".format(task_id, self.PARAM["ROBOT_ID"], task_robot_type))

        task_time = task_data[self.PARAM["START_TIME"]]
        task_time_type = type(task_time)
        if task_time_type != str and task_time is not None:
            raise WrongTaskInputData("Task id: {}. Param '{}' should be str or None type but {} "
                                     "was given.".format(task_id, self.PARAM["START_TIME"], task_time_type))

        task_beh_index_type = type(task_data[self.PARAM["CURRENT_BEH_ID"]])
        if task_beh_index_type != int:
            raise WrongTaskInputData("Task id: {}. Param '{}' should be int type but {} "
                                     "was given.".format(task_id, self.PARAM["CURRENT_BEH_ID"], task_beh_index_type))

        task_status = task_data[self.PARAM["STATUS"]]
        task_status_type = type(task_data[self.PARAM["STATUS"]])
        if task_status_type != str and task_status is not None:
            raise WrongTaskInputData("Task id: {}. Param '{}' should be str or None type but {} "
                                     "was given.".format(task_id, self.PARAM["STATUS"], task_status_type))

        weight_id_type = type(task_data[self.PARAM["WEIGHT"]])
        if weight_id_type not in [int, float]:
            raise WrongTaskInputData("Task id: {}. Param '{}' should be int, float, None type but {} was given."
                                     .format(task_id, self.PARAM["WEIGHT"], weight_id_type))

        behaviour_type = type(task_data[self.PARAM["BEHAVIOURS"]])
        if behaviour_type != list:
            raise WrongTaskInputData("Task id: {}. Param '{}' should be list type but {} was given."
                                     .format(task_id, self.PARAM["BEHAVIOURS"], behaviour_type))

        # sprawdzenie poprawnosci danych
        if task_status not in self.STATUS_LIST.values():
            raise WrongTaskInputData("Task id: {}. '{}' doesn't exist. {} "
                                     "was given.".format(task_id, self.PARAM["STATUS"], task_status))
        if robot_id is None and task_data[self.PARAM["STATUS"]] != self.STATUS_LIST["TO_DO"]:
            raise WrongTaskInputData("Task id: {}. Param '{}' should be set when task was started. Status different"
                                     " than '{}'.".format(task_id, self.PARAM["ROBOT_ID"],
                                                          self.STATUS_LIST["TO_DO"]))

        max_n_beh = len(task_data[self.PARAM["BEHAVIOURS"]]) - 1
        if (task_data[self.PARAM["CURRENT_BEH_ID"]] < -1) or (task_data[self.PARAM["CURRENT_BEH_ID"]] > max_n_beh):
            raise WrongTaskInputData("Task id: {}. Param '{}' should be int in range [-1,{}] but was '{}'"
                                     "".format(task_id, self.PARAM["CURRENT_BEH_ID"], max_n_beh,
                                               task_data[self.PARAM["CURRENT_BEH_ID"]]))

        try:
            datetime.strptime(task_time, "%Y-%m-%d %H:%M:%S")
        except:
            raise WrongTaskInputData("Task id: {}. Param '{}' wrong type. Required YYYY-mm-dd HH:MM:SS".
                                     format(task_id, self.PARAM["START_TIME"], task_time_type))
    # TODO
    # sprawdzenie kolejnosci zachowan w zadaniu
    # sprawdzenie czy pierwszym zachowaniem jest goto
    # dla innych typow zadan odpowiedzialnych za wysylanie MSG do robota current beh index odnosi
    # sie do wykonywalnego przez robota zachowania uwzglednianego w planie. Takie zachowania
    # powinny być pominięte na etapie planowania. Kolejnym zachowaniem nie może być zachowanie nieuwzględniane
    # na etapie planowania.

    def get_info(self):
        """
        Wyswietla informacje o zadaniu.
        """
        data = "id: " + str(self.id) + ", robot_id: " + str(self.robot_id) + ",start_tme: " + str(self.start_time)+"\n"
        data += "current beh id: " + str(self.curr_behaviour_id) + ", status: " + str(self.status) +\
                ", weight: " + str(self.weight) + "\n"
        for behaviour in self.behaviours:
            data += behaviour.get_info()
        return data


class TasksManager:
    """
    Klasa odpowiadajaca za analize kolejnych zadan wykonywanych i przydzielonych do robotow. Zawiera liste
    posortowanych zadan po priorytecie wykonania, a nastepnie po czasie.

    Attributes:
        tasks ([Task, Task, ...]): lista posortowanych zadan dla robotow
    """

    def __init__(self, tasks):
        """
        Parameters:
          tasks ([Task, Task, ...) - lista zadan dla robotow
        """
        self.tasks = []
        self.set_tasks(tasks)

    def set_tasks(self, tasks):
        """
        Odpowiada za przekonwertowanie danych wejsciowych i ustawienie zadan dla atrybutu tasks.

        Parameters:
            tasks ([Task, Task, ...) - lista zadan dla robotow
        """
        type_input_data = type(tasks)
        if type_input_data != list:
            raise WrongTaskInputData("Input tasks list should be list but '{}' was given.".format(type_input_data))

        max_priority_value = 0
        all_tasks = copy.deepcopy(tasks)
        for i, task in enumerate(all_tasks):
            task.weight = task.priority
            max_priority_value = task.weight if task.weight > max_priority_value else max_priority_value
            task.index = i + 1


        # odwrocenie wynika pozniej z funkcji sortujacej, zadanie o najwyzszym priorytecie powinno miec
        # najnizsza wartosc liczbowa
        for task in all_tasks:
            task.weight = max_priority_value - task.weight

        # sortowanie zadan po priorytetach i czasie zgłoszenia
        tasks_id = [data.id for data in sorted(all_tasks, key=lambda
                    task_data:(task_data.weight, task_data.index), reverse=False)]

        self.tasks = []
        for i in tasks_id:
            updated_task = [task for task in all_tasks if task.id == i][0]
            # przepisanie wartosci wejsciowej priorytetu dla zachowania kolejnosci
            updated_task.weight = [task.weight for task in tasks if task.id == i][0]

            self.tasks.append(updated_task)

    def remove_tasks_by_id(self, tasks_id):
        """
        Usuwa z listy zadania na podstawie przekazanej listy id zadan, ktore trzeba usunac.

        Parameters:
             tasks_id ([string, string, ...]): lista z kolejnymi id zadan
        """

        for i in tasks_id:
            for task in self.tasks:
                if task.id == i:
                    self.tasks.remove(task)
                    break

    def get_all_unasigned_unstarted_tasks(self):
        """
        Zwraca liste wszystkich nieprzypisanych i nierozpoczetych zadan.

        Returns:
             (list(Task)): lista zadan
        """
        return [task for task in self.tasks if task.robot_id is None and not task.check_if_task_started()]


class Robot:
    """
    Klasa przechowujaca informacje o pojedynczym robocie do ktorego beda przypisywane zadania

    Attributes:
        id (string): id robota
        edge ((string,string)): krawedz na ktorej aktualnie znajduje sie robot
        planning_on (bool): informuje czy robot jest w trybie planownaia
        is_free (bool): informuje czy robot aktualnie wykonuje jakies zachowanie czy nie
        time_remaining (float/None): czas do ukonczenia zachowania
        task (Task): zadanie przypisane do robota
        next_task_edge ((string,string)): informuje o kolejnej krawedzi przejscia ktora nalezy wyslac do robota
        end_beh_edge (bool): informuje czy zachowanie po przejsciu krawedzia zostanie ukonczone
    """
    def __init__(self, robot_data):
        """
        Parameters:
            robot_data ({"id": string, "edge": (string, string), "planningOn": bool, "isFree": bool,
                         "timeRemaining": float/None}): slownik z danymi o robocie
        """
        #self.validate_input(robot_data)
        self.id = robot_data["id"]
        self.edge = robot_data["edge"]
        self.poi_id = robot_data["poiId"]
        self.planning_on = robot_data["planningOn"]
        self.is_free = robot_data["isFree"]
        self.time_remaining = robot_data["timeRemaining"]
        self.task = None
        self.next_task_edge = None
        self.end_beh_edge = None

        self.check_planning_status()

    def validate_input(self, data):
        """
        Parameters:
            data ({"id": string, "edge": (string, string), "planningOn": bool, "isFree": bool,
                         "timeRemaining": float/None}): slownik z danymi o robocie
        """
        # sprawdzenie czy dane wejsciowe sa dobrego typu
        type_data = type(data)
        if type_data != dict:
            raise WrongRobotInputData("Robot input data should be dict but {} was given.".format(type_data))

        # sprawdzenie czy kazdy parametr istnieje
        if "id" not in data:
            raise WrongRobotInputData("Robot 'id' param doesn't exist.")
        robot_id = data["id"]
        if "edge" not in data:
            raise WrongRobotInputData("Robot id: {}. Param 'edge' doesn't exist.".format(robot_id))
        if "planningOn" not in data:
            raise WrongRobotInputData("Robot id: {}. Param 'planningOn' doesn't exist.".format(robot_id))
        if "isFree" not in data:
            raise WrongRobotInputData("Robot id: {}. Param 'isFree' doesn't exist.".format(robot_id))
        if "timeRemaining" not in data:
            raise WrongRobotInputData("Robot id: {}. Param 'timeRemaining' doesn't exist.".format(robot_id))

        # sprawdzenie czy kazdy parametr wejsciowy jest dobrego typu
        type_id = type(data["id"])
        if type_id != str:
            raise WrongRobotInputData("Robot id: {}. Param 'id' should be str type but {} was given."
                                      "".format(robot_id, type_id))
        type_id = type(data["edge"])
        if type_id != tuple and data["edge"] is not None:
            raise WrongRobotInputData("Robot id: {}. Param 'edge' should be tuple or None type but {} was given."
                                      "".format(robot_id, type_id))
        type_id = type(data["planningOn"])
        if type_id != bool:
            raise WrongRobotInputData("Robot id: {}. Param 'planningOn' should be bool type but {} was given."
                                      "".format(robot_id, type_id))
        type_id = type(data["isFree"])
        if type_id != bool:
            raise WrongRobotInputData("Robot id: {}. Param 'isFree' should be bool type but {} was given."
                                      .format(robot_id, type_id))
        type_id = type(data["timeRemaining"])
        if type_id not in [int, float]:
            raise WrongRobotInputData("Robot id: {}. Param 'timeRemaining' should be int or float type but {} "
                                      "was given.".format(robot_id, type_id))

    def get_current_node(self):
        """
        Returns:
             (string): wezel w ktorym znajduje sie robot lub znajdzie po zakonczeniu zachowania
        """
        return None if self.edge is None else self.edge[1]

    def check_planning_status(self):
        """
        TODO
            - przekonwertowac status z robota i okreslic na jego podstawie czy robot jest w trybie planowania czy nie
            - dopisac testy po wprowadzeniu dodatkowych statusow robota na podstawie, ktorych
                podjeta jest decyzja o planowaniu
        Returns:
            (bool): True - robot zajety, False - robot wolny
        """
        return self.planning_on

    def get_current_destination_goal(self):
        """
        Zwraca Id POI do ktorego aktualnie jedzie robot lub wykonuje w nim jakas operacje (dokowanie,
        wait, oddokowanie). Dla nowo przydzielonych zadan pozwala na sprawdzenie do ktorego POI bedzie jechal
        robot.

        Returns:
            (string): id POI z bazy do ktorego aktualnie jedzie lub bedzie jechal robot, jesli nie jest on skierowany do
                      zadnego POI to zwracana jest wartosc None
        """
        return self.task.get_poi_goal() if self.task is not None else None

    def get_info(self):
        """
        Wyswietla informacje o robocie.
        """
        data = "id: " + str(self.id) + ", edge: " + str(self.edge) + ", planning_on: " + str(self.planning_on) + "\n"
        data += "is free: " + str(self.is_free) + ", time remaining: " + str(self.time_remaining) + "\n"
        task_info = self.task.get_info if self.task is not None else None
        data += "task: " + str(task_info) + "\n"
        data += "next edge" + str(self.next_task_edge) + ", end beh: " + str(self.end_beh_edge)
        return data


class RobotsPlanManager:
    """
    Klasa obslugujaca przypisywanie zadan do robotow i wyciaganie informacji o robotach niezbednych do planowania zadan.

    Attributes:
        robots ({"id": Robot, "id": Robot, ...}): slownik z lista robotow do ktorych beda przypisywane zadania

    """
    def __init__(self, robots, base_poi_edges):
        """
        Parameters:
             robots ({ "id":Robot, "id": Robot, ...]): przekazana lista robotow do planowania zadan
             base_poi_edges ({poi_id: graph_edge(tuple), ...}) : lista z krawedziami bazowymi do ktorych nalezy
                przypisac robota, jesli jest on w POI
        """
        self.robots = robots
        self.set_robots(robots, base_poi_edges)

    def set_robots(self, robots, base_poi_edges):
        """
        Parameters:
             robots ({ "id":Robot, "id": Robot, ...]): przekazana lista robotow do planowania zadan
            base_poi_edges ({poi_id: graph_edge(tuple), ...}) : lista z krawedziami bazowymi do ktorych nalezy
                przypisac robota, jesli jest on w POI
        """
        self.robots = {}
        for i in robots:
            robot = robots[i]
            if robot.planning_on:
                if type(robot.edge) is not tuple:
                    # zamiast krawedzi jest POI TODO pobrania POI z innego miejsca i wpisanie odpowiedniej
                    # krawedzi, jesli nie jest ona znana dla danego robota.
                    # TODO weryfikacja czy dla danego poi istnieje krawedz na grafie, istnieje w podanym slowniku
                    # wejsciowym
                    #print("robot edge/poi id: ", robot.edge, robot.poi_id)
                    robot.edge = base_poi_edges[robot.poi_id]
                self.robots[robot.id] = robot

    def get_robot_by_id(self, robot_id):
        """
        Dla podanego id zwraca obiekt Robot.

        Parameters:
            robot_id (string): id robota

        Returns:
            (Robot): informacje o robocie, jeśli nie ma go na liście to None
        """
        if self.check_if_robot_id_exist(robot_id):
            return self.robots[robot_id]
        else:
            return None

    def set_task(self, robot_id, task):
        """
        Przypisuje zadanie dla robota o podanym id. Jeśli go nie ma to blad.
        # TODO walidacja czy juz nie przypisano do jakiegos robota zadania o podanym ID

        Parameters:
            robot_id (string): id robota
            task (Task): zadanie dla robota
        """

        if not self.check_if_robot_id_exist(robot_id):
            raise TaskManagerError("Robot on id '{}' doesn't exist".format(robot_id))

        if robot_id != task.robot_id and task.robot_id is not None:
            raise TaskManagerError("Task is assigned to different robot. Task {} required robot with "
                                   "id {} but {} was given.".format(task.id, task.robot_id, robot_id))
        task.robot_id = robot_id
        self.robots[robot_id].task = task

    def check_if_robot_id_exist(self, robot_id):
        """
        Sprawdza czy robot o podanym id istnieje na liscie do planownaia.
        Parameters:
            robot_id (string): id robota
        Returns:
            (bool): Jesli robot istnieje to True inaczej False.
        """
        return robot_id in self.robots

    def set_next_edge(self, robot_id, next_edge):
        """
        Ustawia kolejna krawedz przejscia dla robota.

        Parameters:
            robot_id (string): id robota
            next_edge ((string,string)): nastepna krawedz jaka ma sie poruszac robot
        """
        if not self.check_if_robot_id_exist(robot_id):
            raise TaskManagerError("Robot on id '{}' doesn't exist".format(robot_id))
        if self.robots[robot_id].task is None:
            raise TaskManagerError("Can not assign next edge when robot {} doesn't have task.".format(robot_id))
        self.robots[robot_id].next_task_edge = next_edge

    def set_end_beh_edge(self, robot_id, end_beh_edge):
        """
        Ustawia informacje o tym czy dane przejscie krawedzia bedzie konczylo zachowanie w zadaniu czy nie.

        Parameters:
            robot_id (string): id robota
            end_beh_edge (bool): informacja o tym czy jest to koniec zachowania czy nie
        """
        if not self.check_if_robot_id_exist(robot_id):
            raise TaskManagerError("Robot on id '{}' doesn't exist".format(robot_id))
        if self.robots[robot_id].task is None:
            raise TaskManagerError("Can not set end behaviour edge when robot {} doesn't have task.".format(robot_id))
        if self.robots[robot_id].next_task_edge is None:
            raise TaskManagerError("Can not set end behaviour edge when robot {} doesn't have next_task_edge."
                                   "".format(robot_id))
        self.robots[robot_id].end_beh_edge = end_beh_edge

    def get_free_robots(self):
        """
        Zwraca liste robotow do ktorych zadania nie zostaly przypisane. Atrybut robota 'task' jest None.

        Returns:
            ([Robot, Robot, ... ]): lista robotow, ktore nie posiadaja przypisanych zadan.
        """
        return [robot for robot in self.robots.values() if robot.task is None]

    def get_busy_robots(self):
        """
        Zwraca liste robotow do ktorych zadania zostaly przypisane.

        Returns:
            ([Robot, Robot, ... ]): lista robotow, wykonujacych zadania
        """
        return [robot for robot in self.robots.values() if robot.task is not None]

    def get_robots_id_on_given_edges(self, edges):
        """
        Zwraca liste z id robotow, ktore znajduja sie na podanych krawedziach

        Parameters:
            edges ([(string,string), (string,string), ... ]): lista krawedzi na ktorych maja byc znalezione wszystkie
                roboty

        Returns:
            ([string,string, ...]): lista z id robotow znajdujacych sie na wszystkich wskazanych krawedziach
        """
        return [robot.id for robot in self.robots.values() if robot.edge in edges]

    def get_robots_id_on_future_edges(self, edges):
        """
        Zwraca liste z id robotow, ktore znajduja sie na podanych krawedziach. Roboty bez zadan zostaja pominiete.

       Parameters:
            edges ([(string,string), (string,string), ... ]): lista krawedzi na ktorych maja byc znalezione wszystkie
                roboty

        Returns:
            ([string,string, ...]): lista z id robotow znajdujacych sie na wszystkich wskazanych krawedziach
        """
        return [robot.id for robot in self.robots.values() if robot.next_task_edge in edges]

    def get_current_robots_goals(self):
        """
        Zwraca slownik robotow wraz z POI do ktorych aktualnie jada roboty

        Returns:
            ({robot_id: poi_id, ...}): lista robotow z POI aktualnego celu
        """
        busy_robots = {}
        for robot in self.robots.values():
            if robot.task is not None:
                busy_robots[robot.id] = robot.task.get_poi_goal()
        return busy_robots


class PoisManager:
    """
    Zawiera informacje o POI przypisanych do grafu.

    Attributes:
        pois ({poi_id: {"type": typ_poi}, ... ): slownik poi wraz z ich typem
    """
    def __init__(self, graph_data):
        """
        Parameters:
             graph_data (SupervisorGraphCreator): rozszerzony graf do planowania
        """
        self.pois = {}
        self.set_pois(graph_data)

    def set_pois(self, graph):
        """
        Na podstawie danych z grafu tworzy slownik POI przypisanych do wezlow grafu.

        Parameters:
             graph (SupervisorGraphCreator): rozszerzony graf do planowania
        """
        for i in graph.source_nodes:
            node = graph.source_nodes[i]
            if node["poiId"] != "0":  # TODO zweryfikowac wartosc dla wezla bez POI
                self.pois[node["poiId"]] = node["type"]

    def check_if_queue(self, poi_id):
        """
        Sprawdza czy podane POI jest POI kolejkowania.

        Parameters:
            poi_id (string): id poi z bazy

        Returns:
            (bool): informacja czy poi jest typu kolejkownaia (queue)
        """
        if poi_id not in self.pois:
            raise PoisManagerError("Poi id '{}' doesn't exist".format(poi_id))
        return self.pois[poi_id] == gc.base_node_type["queue"]

    def get_raw_pois_dict(self):
        """
        Zwraca pusta slownik z id poi

        Returns:
            ({poi_id: None, poi_id: None, ...}): pusty slownik z POI
        """
        poi_list = {}
        for poi_id in self.pois:
            poi_list[poi_id] = None
        return poi_list

    def get_type(self, poi_id):
        """
        Zwraca typ POI na podstawie id.

        Parameters:
            poi_id (string): id poi z bazy

        Returns:
            (gc.base_node_type[]): typ POI
        """
        if poi_id not in self.pois:
            raise PoisManagerError("Poi id '{}' doesn't exist".format(poi_id))
        return self.pois[poi_id]


class PlanningGraph:
    """
    Klasa do obslugi grafu planujacego.

    Attributes:
        graph (DiGraph): dane o grafie z klasy SupervisorGraphCreator
    """
    def __init__(self, graph):
        """
        Parameters:
            graph (GraphCreator):
        """
        self.graph = copy.deepcopy(graph.graph)
        self.pois = PoisManager(graph).pois

    def block_other_pois(self, robot_node, target_node):
        """
        Funkcja odpowiada za zablokowanie krawedzi grafu zwiazanych z innymi POI niz aktualnym (jesli jest w POI) i
        docelowym.

        Parameters:
            robot_node (string): wezel grafu z supervisora w ktorym aktualnie jest robot
            target_node (string): wezel grafu z supervisora do ktorego zmierza robot

        # TODO do przerobki lista nieblokujacych POI, zamiast 0 ma byc wartosc dla krawedzi niezwiazanych z POI np None
        """
        no_block_poi_ids = ["0", self.graph.nodes[robot_node]["poiId"], self.graph.nodes[target_node]["poiId"]]
        for edge in self.graph.edges(data=True):
            start_node_poi_id = self.graph.nodes[edge[0]]["poiId"]
            end_node_poi_id = self.graph.nodes[edge[1]]["poiId"]
            if start_node_poi_id in no_block_poi_ids and end_node_poi_id in no_block_poi_ids:
                self.graph.edges[edge[0], edge[1]]["planWeight"] = self.graph.edges[edge[0], edge[1]]["weight"]
            else:
                self.graph.edges[edge[0], edge[1]]["planWeight"] = None

    def get_end_go_to_node(self, poi_id, poi_type):
        """
        Zwraca węzeł końcowy krawędzi dla zadania typu GOTO POI.

        Parameters:
            poi_id (string): id POI z bazy
            poi_type (gc.base_node_type["nazwa_typu"]): typ poi

        Returns:
            (string): koncowy wezel krawedzi dojazdu do wlasciwego stanowiska
        """
        if poi_id not in self.pois:
            raise PlaningGraphError("POI {} doesn't exist on graph.".format(poi_id))
        if poi_type["nodeSection"] == gc.base_node_section_type["dockWaitUndock"]:
            return [node[0] for node in self.graph.nodes(data=True) if node[1]["poiId"] == poi_id
                    and node[1]["nodeType"] == gc.new_node_type["dock"]][0]
        elif poi_type["nodeSection"] == gc.base_node_section_type["waitPOI"]:
            return [node[0] for node in self.graph.nodes(data=True) if node[1]["poiId"] == poi_id
                    and node[1]["nodeType"] == gc.new_node_type["wait"]][0]
        else:
            return [node[0] for node in self.graph.nodes(data=True) if node[1]["poiId"] == poi_id][0]

    def get_end_docking_node(self, poi_id):
        """
        Zwraca węzeł końcowy krawędzi dla zadania typu DOCK POI.

        Parameters:
            poi_id (string): id POI z bazy

        Returns:
            (string): koncowy wezel krawedzi zwiazanej z zachowaniem dokowania
        """
        if poi_id not in self.pois:
            raise PlaningGraphError("POI {} doesn't exist on graph.".format(poi_id))
        if self.pois[poi_id]["nodeSection"] != gc.base_node_section_type["dockWaitUndock"]:
            raise PlaningGraphError("POI {} should be one of docking type.".format(poi_id))
        poi_node = [node for node in self.graph.nodes(data=True) if node[1]["poiId"] == poi_id
                    and node[1]["nodeType"] == gc.new_node_type["wait"]]
        return poi_node[0][0]

    def get_end_wait_node(self, poi_id, poi_type):
        """
        Zwraca węzeł końcowy krawędzi dla zadania typu WAIT POI.

        Parameters:
            poi_id (string): id POI z bazy
            poi_type (gc.base_node_type["nazwa_typu"]): typ POI

        Returns:
            (string): koncowy wezel krawedzi zwiazanej z zachowaniem WAIT
        """
        if poi_id not in self.pois:
            raise PlaningGraphError("POI {} doesn't exist on graph.".format(poi_id))
        if self.pois[poi_id]["nodeSection"] not in [gc.base_node_section_type["dockWaitUndock"],
                                                    gc.base_node_section_type["waitPOI"]]:
            raise PlaningGraphError("POI {} should be one of docking/wait POI.".format(poi_id))
        poi_node = None
        if poi_type["nodeSection"] == gc.base_node_section_type["dockWaitUndock"]:
            poi_node = [node for node in self.graph.nodes(data=True) if node[1]["poiId"] == poi_id
                        and node[1]["nodeType"] == gc.new_node_type["undock"]]
        elif poi_type["nodeSection"] == gc.base_node_section_type["waitPOI"]:
            poi_node = [node for node in self.graph.nodes(data=True) if node[1]["poiId"] == poi_id
                        and node[1]["nodeType"] == gc.new_node_type["end"]]
        return poi_node[0][0]

    def get_end_undocking_node(self, poi_id):
        """
        Zwraca węzeł końcowy krawędzi dla zadania typu UNDOCK POI.

        Parameters:
            poi_id (string): id POI z bazy

        Returns:
            (string): koncowy wezel krawedzi zwiazanej z zachowaniem UNDOCK
        """
        if poi_id not in self.pois:
            raise PlaningGraphError("POI {} doesn't exist on graph.".format(poi_id))
        if self.pois[poi_id]["nodeSection"] != gc.base_node_section_type["dockWaitUndock"]:
            raise PlaningGraphError("POI {} should be one of docking/wait POI.".format(poi_id))
        poi_node = [node for node in self.graph.nodes(data=True) if node[1]["poiId"] == poi_id
                    and node[1]["nodeType"] == gc.new_node_type["end"]]

        return poi_node[0][0]

    def get_max_allowed_robots_using_pois(self):
        """
        Zwraca liste zawierajaca maksymalna liczbe robotow, ktora moze byc obslugiwana przez dany typ POI.
        Przekroczenie tej liczby oznacza, ze roboty moga zaczac sie kolejkowac na glownym szlaku komunikacyjnym.

        Returns:
            ({poiId: string, poiId2: string,...}): Slownik z liczba robotow dla ktorego kluczem jest ID POI z bazy a
            maksymalna liczba robotow jaka moze oczekiwac i byc obslugiwana przy stanowisku.
        """
        max_robot_pois = {i: 0 for i in self.pois}
        connected_edges = [edge for edge in self.graph.edges(data=True) if "connectedPoi" in edge[2]]
        for edge in connected_edges:
            poi_id = edge[2]["connectedPoi"]
            poi_type = self.pois[poi_id]
            if poi_type == gc.base_node_type["parking"]:
                # dla POI parkingowych tylko 1 robot
                max_robot_pois[poi_id] = 1
            elif poi_type == gc.base_node_type["queue"]:
                # dla POI z kolejkowaniem tylko tyle robotów ile wynika z krawędzi oczekiwania
                max_robots = edge[2]["maxRobots"]
                max_robot_pois[poi_id] = max_robots if max_robots > 0 else 1
            elif poi_type["nodeSection"] in [gc.base_node_section_type["waitPOI"],
                                             gc.base_node_section_type["dockWaitUndock"]] \
                    and edge[2]["edgeGroupId"] == 0:
                # 1 dla obsługi samego stanowiska + maksymalna liczba robotów na krawędzi związana z danym POI
                max_robots = edge[2]["maxRobots"] + 1
                max_robot_pois[poi_id] = max_robots if max_robots > 0 else 1
        return max_robot_pois

    def get_group_id(self, edge):
        """
        Zwraca id grupy do ktorej przypisana jest krawedz. Numer grupy 0 odnosi sie do krawedzi, ktore nie sa od
        siebie wzajemnie zalezne.

        Parameters:
            edge((string,string)): krawedz grafu

        Returns:
            (int): id grupy krawedzi, 0 dla krawedzi nie wchodzacych w sklad grupy
        """
        return self.graph.edges[edge]["edgeGroupId"]

    def get_robots_in_group_edge(self, edge):
        """
        Zwraca liste z id robotow, ktore przynaleza do danej krawedzi lub liste robotow z calej grupy, jesli nalezy
        ona do grupy. Dla niezerowych grup liczba przypisanych robotow do krawedzi grafu musi być 0 lub 1.

        Parameters:
            edge (string,string): krawedz dla ktorej maja byc zwrocone roboty

        Returns:
            ([string, string, ... ]): lista id robotow, ktore przypisane sa do danej krawedzi lub jesli krawedz stanowi
                grupe to zwracane sa wszystkie roboty nalezace do grupy.
        """
        robots_ids = []
        group_id = self.get_group_id(edge)
        if group_id != 0:
            # krawedz nalezy do grupy
            for edge_data in self.graph.edges(data=True):
                if self.get_group_id((edge_data[0], edge_data[1])) == group_id:
                    robots_ids = robots_ids + edge_data[2]["robots"]
            if len(robots_ids) > 1:
                raise PlaningGraphError("Only 1 robot can be on edge '{}' belongs to group '{}'".format(edge, group_id))
        else:
            robots_ids = self.graph.edges[edge]["robots"]
            if len(robots_ids) > self.graph.edges[edge]["maxRobots"]:
                raise PlaningGraphError("Max allowed robots on edge ({},{}) is {} but was given {}.".format(
                    edge[0], edge[1], self.graph.edges[edge]["maxRobots"], len(robots_ids)))
        return robots_ids

    def get_edges_by_group(self, group_id):
        """
        Zwraca liste krawedzi dla grupy o danym id.

        Parameters:
            group_id (int): id grupy krawedzi

        Returns:
            ([(string,string), (string,string), ... ]): lista krawedzi nalezaca do podanej grupy
        """
        return [(edge[0], edge[1]) for edge in self.graph.edges(data=True) if edge[2]["edgeGroupId"] == group_id]

    def get_max_allowed_robots(self, edge):
        """
        Zwraca maksymalna dozwolona liczbe robotow dla danej krawedzi. Jesli krawedz nalezy do grupy to 1 robot.

        Attributes:
            edge ((string,string)): dana krawedz

        Returns:
              (int): maksymalna liczba robotow jaka moze znajdowac sie na danej krawedzi
        """
        return 1 if self.get_group_id(edge) != 0 else self.graph.edges[edge]["maxRobots"]

    def get_poi(self, edge):
        """
        Dla podanej krawedzi zwraca zwiazane z nia POI, jesli takie istnieje.

        Parameters:
            edge (string,string): krawedz grafu

        Returns:
            (string): zwraca id poi, jesli krawedz zwiazana jest z POI, jesli nie to None
        """
        poi_node = self.graph.nodes[edge[1]]['poiId']
        if poi_node != "0":  # TODO weryfikacja dla jakich wartosci poi node wezel nie ma poi None czy "0"
            return poi_node
        else:
            return None

    def get_path(self, start_node, end_node):
        """
        Zwraca sciezke od aktualnego polozenia robota do celu.
        TODO - obsluga gry wezel poczatkowy jest tym samym co koncowy np. ponowne zlecenie dojazdu tego samego
        robota do parkingu
        Parameters:
            start_node (string): wezel od ktorego ma byc rozpoczete planowanie
            end_node (string): wezel celu

        Returns:
            (list): kolejne krawedzie grafu, ktorymi ma sie poruszac robot, aby dotrzec do celu
        """
        self.block_other_pois(start_node, end_node)
        if start_node == end_node:
            raise PlaningGraphError("Wrong plan. Start node '{}' should be different than end node '{}'."
                                    .format(start_node, end_node))
        return nx.shortest_path(self.graph, source=start_node, target=end_node, weight='planWeight')

    def get_path_length(self, start_node, end_node):
        """
        Zwraca wage dojazdu do punktu.

        Parameters:
            start_node (string): wezel od ktorego ma byc rozpoczete planowanie
            end_node (string): wezel celu

        Returns:
            (float): czas dojazdu od start_node do end_node
        """
        self.block_other_pois(start_node, end_node)
        if start_node == end_node:
            return 0
        else:
            return nx.shortest_path_length(self.graph, source=start_node, target=end_node, weight='planWeight')

    def get_base_pois_edges(self):
        """
        Returns:
            ({poi_id: graph_edge(tuple), ...}) : lista z krawedziami bazowymi do ktorych nalezy
             przypisac robota, jesli jest on w POI
        """
        base_poi_edges = {}
        for poi_id in self.pois:
            poi_nodes = [node[0] for node in self.graph.nodes(data=True) if node[1]["poiId"] == poi_id]
            if len(poi_nodes) == 1:
                # poi jest typu parking lub queue
                edges = [edge for edge in self.graph.edges(data=True) if edge[1] in poi_nodes][0]
                base_poi_edges[poi_id] = (edges[0], edges[1])
            elif len(poi_nodes) == 4:
                # poi z dokowaniem
                start_node = [node for node in poi_nodes
                              if self.graph.nodes[node]["nodeType"] == gc.new_node_type["undock"]]
                end_node = [node for node in poi_nodes if self.graph.nodes[node]["nodeType"] == gc.new_node_type["end"]]
                base_poi_edges[poi_id] = (start_node[0], end_node[0])
            elif len(poi_nodes) == 2:
                # poi bez dokowania
                start_node = [node for node in poi_nodes
                              if self.graph.nodes[node]["nodeType"] == gc.new_node_type["wait"]]
                end_node = [node for node in poi_nodes if self.graph.nodes[node]["nodeType"] == gc.new_node_type["end"]]
                base_poi_edges[poi_id] = (start_node[0], end_node[0])
            else:
                raise PlaningGraphError("Input graph wrong structure.")
        return base_poi_edges


class Dispatcher:
    """
    Klasa dispatchera odpowiadajaca za tworzenie planu, przydzielanie zadan do robotow i unikanie drog
    kolizyjnych na grafie. Wolne roboty blokujace stanowiska kierowane sa odpowiednio na miejsca oczekiwan
    queue/parking.

    Attributes:
        planning_graph (PlanningGraph): graf do planowania zadan
        pois (PoisManager): manager poi, zawiera m.in. informacje o ich typach
        robots_plan (RobotsPlanManager): zawiera informacje o robotach, zadaniach i kolejnych akcjach wynikajacych z
            planownaia
        unanalyzed_tasks_handler (TasksManager): zawiera informacje o zadaniach, ktore nalezy przeanalizowac
            i przypisac do robotow
    """

    def __init__(self, graph_data, robots):
        """
        Utworzenie klasy Dispatcher'a i ustawienie wartosci atrybutow planning_graph, pois, robots_plan.

        Parameters:
            graph_data (SupervisorGraphCreator): wygenerwany rozszerzony graf do planowania kolejnych zachowan robotow
            robots ({"id": Robot, "id": Robot, ...}): slownik z lista robotow do ktorych beda przypisywane zadania

        TODO
            - wchodzi lokalizacja robota, jako edge moze wejsc id POI, które należy zastąpić odpowiednią krawędzią na
            której jest robot
        """
        self.planning_graph = PlanningGraph(graph_data)
        self.pois = PoisManager(graph_data)
        
        self.robots_plan = RobotsPlanManager(robots, self.planning_graph.get_base_pois_edges())
        self.init_robots_plan(robots)

        self.unanalyzed_tasks_handler = None

    def get_plan_all_free_robots(self, graph_data, robots, tasks):
        """
        Zwraca liste robotow wraz z zaplanowanymi krawedziami przejscia na grafie, ktore aktualnie moga zostac wykonane

        Parameters:
            graph_data (SupervisorGraphCreator): wygenerwany rozszerzony graf do planowania kolejnych zachowan robotow
            robots ({"id": Robot, "id": Robot, ...}): slownik z lista robotow do ktorych beda przypisywane zadania
            tasks ([Task, Task, ...]): lista posortowanych zadan dla robotow

        Returns:
             ({robotId: {"taskId": string, "nextEdge": (string,string)/None, "endBeh": boolean/None},...})
              - plan dla robotow, ktory moze zostac od razu zrealizowany, gdyz nie ma kolizji; jesli niemożliwe jest
                zlecenie kolejnej krawedzi, bo jest zablokowana to None
        """
        self.planning_graph = PlanningGraph(graph_data)
        self.set_plan(robots, tasks)

        plan = {}  # kluczem jest id robota
        for robot_plan in self.robots_plan.get_busy_robots():
            if robot_plan.next_task_edge is not None:
                plan[robot_plan.id] = {"taskId": robot_plan.task.id, "nextEdge": robot_plan.next_task_edge,
                                       "endBeh": robot_plan.end_beh_edge}
        return plan

    def get_plan_selected_robot(self, graph_data, robots, tasks, robot_id):
        """
        Zwraca liste robotow wraz z zaplanowanymi krawedziami przejscia na grafie, ktore aktualnie moga zostac
        wykonane
        Parameters:
            graph_data (SupervisorGraphCreator): wygenerwany rozszerzony graf do planowania kolejnych zachowan robotow
            robots ({"id": Robot, "id": Robot, ...}): slownik z lista robotow do ktorych beda przypisywane zadania
            tasks ([Task, Task, ...]): lista posortowanych zadan dla robotow
            robot_id (string): id robota dla ktorego ma byc zwrocony plan

        Returns:
             ({"taskId": string, "nextEdge": (string,string)/None, "endBeh": boolean/None},...}): plan dla robotow,
             ktory moze zostac od razu zrealizowany, gdyz nie ma kolizji, w preciwnym wypadku None
        """
        self.planning_graph = PlanningGraph(graph_data)
        # print("tasks len: ", str(len(tasks)))
        #for task in tasks:
        #    task.print_info()
        self.set_plan(robots, tasks)

        given_robot = self.robots_plan.get_robot_by_id(robot_id)
        # print("robot")
        # print(given_robot.task.id)
        # print(given_robot.next_task_edge)
        if given_robot.next_task_edge is None:
            return None
        else:
            return {"taskId": given_robot.task.id, "nextEdge": given_robot.next_task_edge,
                    "endBeh": given_robot.end_beh_edge}

    def set_plan(self, robots, tasks):
        """
        Ustawia plan dla robotow.

        Parameters:
            robots ({"id": Robot, "id": Robot, ...}): slownik z lista robotow do ktorych beda przypisywane zadania
            tasks ([Task, Task, ...]): lista posortowanych zadan dla robotow
        """
        self.set_tasks(tasks)
        self.init_robots_plan(robots)
        self.set_tasks_doing_by_robots()
        self.set_task_assigned_to_robots()
        self.set_other_tasks()

    def init_robots_plan(self, robots):
        """Ustawia roboty aktywnie dzialajace w systemie tzn. podlegajace planowaniu i przydzielaniu zadan.

        Parameters:
            robots ({"id": Robot, "id": Robot, ...}): slownik z lista robotow do ktorych beda przypisywane zadania
        """
        self.robots_plan = RobotsPlanManager(robots, self.planning_graph.get_base_pois_edges())

    def set_tasks(self, tasks):
        """
        Ustawia menadzera nieprzeanalizowanych zadan w ramach planowania.

        Parameters:
            tasks ([Task, Task, ...]): lista posortowanych zadan dla robotow
        """
        self.unanalyzed_tasks_handler = TasksManager(tasks)

    def set_tasks_doing_by_robots(self):
        """
        Przypisanie zadan do robotow, ktore aktualnie pracuja i usuniecie ich z listy zadan do przeanalizowania.
        """
        # przypisanie zadan z POI w których aktualnie jest robot
        tasks_id_to_remove = []
        for unanalyzed_task in self.unanalyzed_tasks_handler.tasks:
            current_behaviour_is_goto = unanalyzed_task.get_current_behaviour().check_if_go_to()
            task_started = unanalyzed_task.check_if_task_started()
            for robot in self.robots_plan.get_free_robots():
                if robot.id == unanalyzed_task.robot_id and task_started and not current_behaviour_is_goto:
                    self.robots_plan.set_task(robot.id, unanalyzed_task)
                    self.set_task_edge(robot.id)
                    tasks_id_to_remove.append(unanalyzed_task.id)
        self.unanalyzed_tasks_handler.remove_tasks_by_id(tasks_id_to_remove)

        # przypisanie zadan z POI do ktorych aktualnie jedzie robot
        tasks_id_to_remove = []
        for unanalyzed_task in self.unanalyzed_tasks_handler.tasks:
            current_behaviour_is_goto = unanalyzed_task.get_current_behaviour().check_if_go_to()
            task_poi_goal = unanalyzed_task.get_poi_goal()
            task_started = unanalyzed_task.check_if_task_started()
            for robot in self.robots_plan.get_free_robots():
                robot_poi = self.planning_graph.get_poi(robot.edge)
                if (robot.id == unanalyzed_task.robot_id and task_started and current_behaviour_is_goto)\
                        and (robot_poi == task_poi_goal or robot_poi is None):
                    self.robots_plan.set_task(robot.id, unanalyzed_task)
                    self.set_task_edge(robot.id)
                    tasks_id_to_remove.append(unanalyzed_task.id)
        self.unanalyzed_tasks_handler.remove_tasks_by_id(tasks_id_to_remove)

        # przypisanie pozostalych zadan rowniez wykonywanych przez agv bedace w POI i majace jechac do innego
        # jesli kolejne POI docelowe ma wolna przestrzen do obslugi to robot wysylany jest do tego POI
        free_slots_in_poi = self.get_free_slots_in_pois()
        tasks_id_to_remove = []
        for unanalyzed_task in self.unanalyzed_tasks_handler.tasks:
            task_poi_goal = unanalyzed_task.get_poi_goal()
            task_started = unanalyzed_task.check_if_task_started()
            for robot in self.robots_plan.get_free_robots():
                robot_poi = self.planning_graph.get_poi(robot.edge)
                if robot.id == unanalyzed_task.robot_id and task_started and robot_poi != task_poi_goal:
                    self.robots_plan.set_task(robot.id, unanalyzed_task)
                    if free_slots_in_poi[task_poi_goal] > 0:
                        free_slots_in_poi[task_poi_goal] = free_slots_in_poi[task_poi_goal] - 1
                        self.set_task_edge(robot.id)
                    tasks_id_to_remove.append(unanalyzed_task.id)

        self.unanalyzed_tasks_handler.remove_tasks_by_id(tasks_id_to_remove)

    def set_task_assigned_to_robots(self):
        """
        Przypisanie do wolnych robotow zadan, ktore sa do nich przypisane. Po przypisaniu zadania
        usuwane jest ono z menadzera nieprzeanalizowanych zadan (unanalyzed_tasks_handler).
        """
        tasks_id_to_remove = []
        for unanalyzed_task in self.unanalyzed_tasks_handler.tasks:
            for robot in self.robots_plan.get_free_robots():
                if robot.task is None:
                    if unanalyzed_task.robot_id == robot.id and not unanalyzed_task.check_if_task_started():
                        self.robots_plan.set_task(robot.id, unanalyzed_task)
                        self.set_task_edge(robot.id)
                        tasks_id_to_remove.append(unanalyzed_task.id)

        self.unanalyzed_tasks_handler.remove_tasks_by_id(tasks_id_to_remove)

    def set_other_tasks(self):
        """
        Przypisanie zadan do pozostalych robotow z uwzglednieniem pierwszenstwa przydzialu zadan do robotow
        blokujacych POI.
        """
        init_time = time.time()
        while True:

            free_robots_id = [robot.id for robot in self.robots_plan.get_free_robots()]
            blocking_robots_id = self.get_robots_id_blocking_poi()

            n_free_robots = len(free_robots_id)
            n_blocking_robots = len(blocking_robots_id)

            all_robots_tasks = self.get_free_task_to_assign(n_free_robots)
            blocking_robots_tasks = self.get_free_task_to_assign(n_blocking_robots)

            n_all_tasks = len(all_robots_tasks)
            n_blocking_robots_tasks = len(blocking_robots_tasks)

            if n_all_tasks == n_free_robots and n_all_tasks != 0:
                self.assign_tasks_to_robots(all_robots_tasks, free_robots_id)
                # przypisywanie zadań zakończone, bo każdy aktywny w systemie robot powinien już mieć zadanie
                break
            elif n_blocking_robots > 0 and n_blocking_robots_tasks != 0:
                self.assign_tasks_to_robots(blocking_robots_tasks, blocking_robots_id)
            elif n_free_robots > 0 and n_all_tasks != 0:
                # nie do wszystkich robotow beda przypisane zadania, dlatego najpierw trzeba sprawdzic i
                # przypisać zadania do robotów blokujących POI
                self.assign_tasks_to_robots(all_robots_tasks, free_robots_id)
            else:
                # wysłanie pozostałych blokujących robotów na parkingi
                if n_blocking_robots != 0:
                    # jeśli jakieś roboty dalej blokują POI po przypisaniu zadań to wysyłane są do
                    # najbliższych prakingów/kolejek.
                    # weryfikacja czy nie ma robotów blokujących -> jeśli są to wysłać je do kolejki (roboty bez zadań)
                    # na parkingach raczej powinny być roboty wykonujące zadania, które nie mogą podjechać do stanowiska
                    self.send_free_robots_to_parking(blocking_robots_id)
                break

            current_time = time.time()
            if (current_time-init_time) > 5:
                raise TimeoutPlanning("Set other tasks loop is too long.")

    def set_task_edge(self, robot_id):
        """
        Ustawia kolejna krawedz przejscia dla robota o danym id.

        Parameters:
            robot_id (string): id robota dla ktorego ma zostac dokonana aktualizacja przejscia po krawedzi
        """
        robot = self.robots_plan.get_robot_by_id(robot_id)
        if robot.planning_on and robot.is_free:
            # robot wykonal fragment zachowania lub ma przydzielone zupelnie nowe zadanie
            # weryfikacja czy kolejna akcja jazdy krawedzia moze zostac wykonana
            start_node = robot.get_current_node()
            end_node = self.get_undone_behaviour_node(robot.task)
            path_nodes = self.planning_graph.get_path(start_node, end_node)
            next_edge = (path_nodes[0], path_nodes[1])

            next_group_id = self.planning_graph.get_group_id(next_edge)

            base_poi_edges = self.planning_graph.get_base_pois_edges()
            poi_id = robot.task.get_poi_goal()
            poi_group_id = self.planning_graph.get_group_id(base_poi_edges[poi_id])
            robot_current_group_id = self.planning_graph.get_group_id(robot.edge)
            poi_group_ids = [self.planning_graph.get_group_id(poi_edge) for poi_edge in base_poi_edges.values()]
            # dostepnosc kolejnego POI
            free_robots_in_poi = []
            for free_robot in self.robots_plan.get_free_robots():
                poi = self.planning_graph.get_poi(free_robot.edge)
                if poi is not None:
                    if not self.pois.check_if_queue(poi) and poi == poi_id:
                        free_robots_in_poi.append(free_robot.id)

            robots_using_poi = free_robots_in_poi
            current_goals = self.robots_plan.get_current_robots_goals()
            max_robots = self.planning_graph.get_max_allowed_robots_using_pois()[poi_id]
            for r_id in current_goals:
                if poi_id == current_goals[r_id]:
                    robots_using_poi.append(r_id)
            robots_using_poi = np.unique(robots_using_poi)
            free_slots = self.get_free_slots_in_pois()[poi_id]
            poi_availability = free_slots > 0 or poi_group_id == robot_current_group_id \
                               or robot_current_group_id not in poi_group_ids  \
                               or (len(robots_using_poi) <= max_robots and robot_id in robots_using_poi)

            if next_group_id != 0:
                group_edges = self.planning_graph.get_edges_by_group(next_group_id)
                planned_robots_ids = self.robots_plan.get_robots_id_on_given_edges(group_edges)
                future_planed_robots_id = self.robots_plan.get_robots_id_on_future_edges(group_edges)
                robots_in_group_edge = self.planning_graph.get_robots_in_group_edge(next_edge)
                robots_ids = [i for i in np.unique(robots_in_group_edge + planned_robots_ids + future_planed_robots_id)]
                if robot.id in robots_ids:
                    robots_ids.remove(robot.id)
                edge_is_available = 0 == len(robots_ids)
            else:
                # krawedz nie nalezy do grupy
                # warunek na sprawdzenie dostepnosci kolejnej krawedzi
                current_robots_ids = self.robots_plan.get_robots_id_on_given_edges([next_edge])
                future_planed_robots_id = self.robots_plan.get_robots_id_on_future_edges([next_edge])
                robots_ids = [i for i in np.unique(current_robots_ids + future_planed_robots_id)]
                if robot.id in robots_ids:
                    robots_ids.remove(robot.id)
                edge_is_available = self.planning_graph.get_max_allowed_robots(next_edge) > len(robots_ids)

            if edge_is_available and poi_availability:
                # jesli krawedz konczy dane zachowanie to ustawiany jest parametr endBehEdge na True
                undone_behaviour = robot.task.get_current_behaviour()
                self.robots_plan.set_next_edge(robot_id, next_edge)
                if undone_behaviour.get_type() != Behaviour.TYPES["goto"]:  # dock,wait,undock -> pojedyncze przejscie
                    # na grafie
                    self.robots_plan.set_end_beh_edge(robot_id, True)
                elif len(path_nodes) == 2:
                    self.robots_plan.set_end_beh_edge(robot_id, True)
                else:
                    self.robots_plan.set_end_beh_edge(robot_id, False)

    def get_robots_id_blocking_poi(self):
        """
        Zwraca liste robotow, ktore dla aktualnego przydzialu zadan sa zablokowane przez roboty bez zadan.

        Returns:
            list ([robotId,...]): lista zawierajaca ID robotow, ktore blokuja POI.
        """
        temp_blocked_pois = self.pois.get_raw_pois_dict()
        for robot in self.robots_plan.get_free_robots():
            poi_id = self.planning_graph.get_poi(robot.edge)
            if poi_id is not None:
                if not self.pois.check_if_queue(poi_id):
                    if temp_blocked_pois[poi_id] is None:
                        temp_blocked_pois[poi_id] = []
                    temp_blocked_pois[poi_id] = temp_blocked_pois[poi_id] + [robot.id]

        robots_goals = self.robots_plan.get_current_robots_goals()
        blocking_robots = []
        for robot_id in robots_goals:
            poi_id = robots_goals[robot_id]
            if temp_blocked_pois[poi_id] is not None:
                blocking_robots = blocking_robots + temp_blocked_pois[poi_id]
        # zwracana jest lista robotów, które blokują POI do których aktualnie jadą roboty
        return blocking_robots

    def get_robots_using_pois(self):
        """
        Zwraca liczbe robotow, ktore aktualnie uzywaja danego POI. Przez uzywanie POI rozumie sie
        dojazd do POI, dokowanie, obsluge w POI lub oddokowanie. Jesli przydzielona krawedz przejscia dotyczy uzycia
        POI to rowniez jest uwzgledniana.

        Returns:
            robotsToPoi ({poiId: string, ...}): Slownik z liczba robotow dla ktorego kluczem jest ID POI z bazy
                a wartoscia liczba robotow zmierzajaca/bedaca do danego POI
        """
        robots_to_poi = self.pois.get_raw_pois_dict()
        for i in robots_to_poi:
            robots_to_poi[i] = 0
        all_robots = self.robots_plan.robots.values()
        for robot in all_robots:
            goal_id = robot.get_current_destination_goal()
            if goal_id is not None:
                robots_to_poi[goal_id] = robots_to_poi[goal_id] + 1
        return robots_to_poi

    def get_free_slots_in_pois(self):
        """
            Funkcja zwraca liste dostepnych slotow dla kazdego z POI.
        Returns:
             ({poi_id: int, ...}) : liczba wolnych slotow dla kazdego poi.
        """
        robots_using_poi = self.get_robots_using_pois()
        free_robots_in_poi = {poi: 0 for poi in self.pois.pois}
        for robot in self.robots_plan.get_free_robots():
            poi_id = self.planning_graph.get_poi(robot.edge)
            if poi_id is not None:
                free_robots_in_poi[poi_id] += 1

        max_robots_in_poi = self.planning_graph.get_max_allowed_robots_using_pois()
        free_slots_in_poi = {}
        for poi_id in robots_using_poi:
            diff = max_robots_in_poi[poi_id] - robots_using_poi[poi_id] - free_robots_in_poi[poi_id]
            free_slots_in_poi[poi_id] = 0 if diff < 0 else diff
        return free_slots_in_poi

    def get_free_task_to_assign(self, robots_numbers):
        """
        Zwraca liste zadan, ktore nalezy przypisac do robotow. Jako argument podawana jest liczba robotow
        ktore maja otrzymac zadania. Nie moze byc ona wieksza niz liczba robotow, ktore nie maja zadan i pracuja
        w trybie autonomicznym.

        Parameters:
            robots_numbers (int): liczba robotow dla ktorych powinny byc wygenerowane zadania

        Returns:
            freeTasks ([Task,Task,...]): lista zadan, ktore nalezy przydzielic do robotow. Liczba zadan moze
                byc mniejsza niz liczba robotow co wynika z: mniejszej liczby zadan niz robotow; nie istnieja
                zadania w systemie, ktorych przypisanie powoduje spelnienie warunku, ze dla danego POI liczba
                przypisanych robotow jest mniejsza od maksymalnej liczby obslugiwanej w danym POI.
        """

        free_slots_in_poi = self.get_free_slots_in_pois()
        # Lista zadań, które nie są przypisane do robotów
        free_tasks = []
        all_free_tasks = self.unanalyzed_tasks_handler.get_all_unasigned_unstarted_tasks()
        for task in all_free_tasks:
            goal_id = task.get_poi_goal()
            if free_slots_in_poi[goal_id] > 0:
                free_slots_in_poi[goal_id] = free_slots_in_poi[goal_id] - 1
                free_tasks.append(task)
                if len(free_tasks) == robots_numbers:
                    break
        return free_tasks

    def assign_tasks_to_robots(self, tasks, robots_id):
        """
        Optymalne przypisanie zadan do robotow pod katem wykonania. W pierwszej kolejnosci zadanie o najwyzszym
        priorytecie dostaje robot, ktory wykona je najszybciej. Po przypisaniu zadania usuwane jest ono z listy
        all_tasks.

        Parameters:
            tasks ([Task, Task, ...]): lista posortowanych zadan ktore maja byc przypisane do robotow
            robots_id ([string, string, ...]): lista id robotow do ktorych maja zostac przypisane zadania
        """
        for task in tasks:
            fastest_robot_id = None
            min_task_time = None
            for r_id in robots_id:
                robot_node = self.robots_plan.get_robot_by_id(r_id).get_current_node()
                target_node = self.get_undone_behaviour_node(task)
                task_time = self.planning_graph.get_path_length(robot_node, target_node)
                if min_task_time is None or min_task_time > task_time:
                    fastest_robot_id = r_id
                    min_task_time = task_time

            # Przypisanie robota do zadania
            self.robots_plan.set_task(fastest_robot_id, task)
            self.set_task_edge(fastest_robot_id)
            robots_id.remove(fastest_robot_id)

        self.unanalyzed_tasks_handler.remove_tasks_by_id([task.id for task in tasks])

    def send_free_robots_to_parking(self, blocking_robots_id):
        """
        Tworzy i przypisuje do robotow blokujacych POI zadanie jazdy do Parkingu/Queue w celu odblokowania POI.

        Parameters:
            blocking_robots_id ([string, string,...]): lista id robotow, ktore blokuja POI

        TODO
            - utworzenie zadan dojazdu w wolne miejsce dla robotow blokujacych POI
            - przypisanie zadan do robotow
        """
        pass

    def send_busy_robots_to_parking(self, blocking_robots_id):
        """
        Tworzy i przypisuje do robotow blokujacych POI zadanie jazdy do Parkingu/Queue w celu odblokowania POI.

        Parameters:
            blocking_robots_id ([string,string,...]): lista id robotow, ktore blokuja POI

        TODO
            - przypisanie zadan do robotow
            - obsluga robotow ktore wykonuja inne zadanie i musza zjechac na parking
        """
        pass

    def get_undone_behaviour_node(self, task):
        """
        Wybiera z zadania pierwsze zachowanie, ktore nie zostalo jeszcze ukonczone, dla zadan nie rozpoczetych
        jest to pierwsze zachowanie jakie wystepuje w zadaniu.

        Parameters:
            task (Task): zadanie dla ktorego okreslany jest wezel do ktorego ma dotrzec robot w zaleznosci
                od kolejnego zadania do wykonania

        Returns:
            goalNode (string): id wezla z grafu rozszerzonego na ktorym opiera sie tworzenie zadan
        """
        poi_id = task.get_poi_goal()
        behaviour = task.get_current_behaviour()
        goalNode = None
        if behaviour.get_type() == Behaviour.TYPES["goto"]:
            goalNode = self.planning_graph.get_end_go_to_node(poi_id, self.pois.get_type(poi_id))
        elif behaviour.get_type() == Behaviour.TYPES["dock"]:
            goalNode = self.planning_graph.get_end_docking_node(poi_id)
        elif behaviour.get_type() == Behaviour.TYPES["wait"] or behaviour.get_type() == Behaviour.TYPES["bat_ex"]:
            goalNode = self.planning_graph.get_end_wait_node(poi_id, self.pois.get_type(poi_id))
        elif behaviour.get_type() == Behaviour.TYPES["undock"]:
            goalNode = self.planning_graph.get_end_undocking_node(poi_id)
        return goalNode
