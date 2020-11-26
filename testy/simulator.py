import graph_creator as gc
from dispatcher import Dispatcher
import networkx as nx
import matplotlib.pyplot as plt
import copy


class Robot:
    """
    Klasa przechowujaca informacje o pojedynczym robocie do ktorego beda przypisywane zadania

    Attributes:
        id (string): id robota
        edge ((int,int)): krawedz na ktorej aktualnie znajduje sie robot
        planning_on (bool): informuje czy robot jest w trybie planownaia
        is_free (bool): informuje czy robot aktualnie wykonuje jakies zachowanie czy nie
        time_remaining (float): czas do ukonczenia zachowania
        beh_duration (float): czas wykonywnaia zachowania
        beh_time (float): czas trwania zachowania
        task_id (int): id przypisanego zadania
        end_beh (bool): czy przejscie jest koncowe, aby wykonac zachowanie skladowae zadania
    """
    def __init__(self, robot_data):
        """
        Parameters:
            robot_data ({"id": int, "edge": (int, int), "planningOn": bool, "isFree": bool, "timeRemaining": float}):
                slownik z danymi o robocie
        """
        self.id = robot_data["id"]
        self.edge = robot_data["edge"]
        self.planning_on = robot_data["planningOn"]
        self.is_free = robot_data["isFree"]
        self.time_remaining = robot_data["timeRemaining"]
        self.beh_duration = 0
        self.beh_time = 0
        self.task_id = None
        self.end_beh = False


class Robots:
    """
    Attributes:
        robots ([{"id": string, "edge": (int, int), "planningOn": bool, "isFree": bool, "timeRemaining": float}
            ,...]): lista z danymi o robocie
    """

    # OK
    def __init__(self, robots, step_time):
        self.step_time = step_time
        self.robots = []
        self.set_robots(robots)

    # OK
    def run(self):
        for robot in self.robots:
            if robot.id is not None:
                robot.beh_duration = robot.beh_duration + self.step_time
                # warunek zakonczenia wykonywania zachowania
                robot.is_free = robot.beh_duration >= robot.beh_time

    # OK
    def set_robots(self, robots):
        """
        Parameters:
            robots ({"id": int, "edge": (int, int), "planningOn": bool, "isFree": bool, "timeRemaining": float}):
                slownik z danymi o robocie
        """
        for robot_data in robots:
            self.robots.append(Robot(robot_data))

    # OK
    def set_tasks(self, tasks_list):
        """
        Attributes:
            tasks_list ([{"robotId": int, "taskId": int, "taskDuration": int, 'nextEdge': (int, int),
                         "endBeh": bool},...])
        """
        for task in tasks_list:
            for robot in self.robots:
                if robot.id == task["robotId"]:
                    robot.task_id = task["taskId"]
                    robot.beh_time = task["taskDuration"]
                    robot.beh_duration = 0
                    robot.is_free = False
                    robot.edge = task["nextEdge"]
                    robot.end_beh = task["endBeh"]
                    break

    # OK
    def get_robots_status(self):
        """
        Zwraca liste robotow wraz ze statusem czy wolny, czy wykonuje zadanie

        Returns:
            ([{"id": int, "isFree": bool, "taskId": int, "edge": (int,int), "endBeh": bool, "planningOn": bool},...])
        """
        robots_list = []
        for robot in self.robots:
            robots_list.append({"id": robot.id, "isFree": robot.is_free, "taskId": robot.task_id,
                                "edge": robot.edge, "endBeh": robot.end_beh, "planningOn": robot.planning_on,
                                "timeRemaining": 0})
        return robots_list

    # OK
    def print_robot_status(self):
        """
        Funkcja wyswietla aktualny stan robotow.
        :return:
        """
        print("---------------------------Robots---------------------------")
        print("{:<9} {:<8} {:<8} {:<12} {:<9} {:<7} {:<6}".format('robotId', 'taskId', 'edge', 'behDuration', 'behTime',
                                                                  'isFree', 'endBeh'))
        for robot in self.robots:
            print("{:<9} {:<6} {:<15} {:<10} {:<7} {:<6} {:<6}".format(str(robot.id), str(robot.task_id),
                                                                       str(robot.edge),
                                                                       str(robot.beh_duration), str(robot.beh_time),
                                                                       str(robot.is_free),
                                                                       str(robot.end_beh)))


# OK
def convert_robots_state_to_dispatcher_format(robots_state_list):
    """
    Parameters:
        robots_state_list ([{"id": int, "isFree": bool, "taskId": int, "edge": (int,int),
        "endBeh": bool, "planningOn": bool},...]) - stan listy robotow z symulatora

    Returns:
         ({"id": int, "edge": (int, int), "planningOn": bool, "isFree": bool, "timeRemaining": float}):
                slownik z danymi o robocie
    """
    robotsList = []
    for robot in robots_state_list:
        robotsList.append({"id": robot["id"], "isFree": robot["isFree"],
                           "edge": robot["edge"], "planningOn": robot["planningOn"],
                           "timeRemaining": robot["timeRemaining"]})
    return robotsList


class Task:
    STATUS_LIST = {  # slownik wartosci nazw statusow zadania
        "TO_DO": 'To Do',  # nowe zadanie, nie przypisane do robota
        "IN_PROGRESS": "IN_PROGRESS",  # zadanie w trakcie wykonywania
        "ASSIGN": "ASSIGN",  # zadanie przypisane, ale nie wykonywane. Oczekiwanie na potwierdzenie od robota
        "DONE": "COMPLETED"  # zadanie zakonczone
    }

    def __init__(self, task_data):
        """
        Attributes:
            task_data ([{"id": int, "behaviours": [Behaviour, Behaviour, ...],
                  "robotId": int, "timeAdded": time, "PRIORITY": Task.PRIORITY["..."]}]): zadanie dla robota
        """
        self.id = task_data["id"]
        self.robot_id = task_data["robot"]
        self.start_time = task_data["start_time"]
        self.behaviours = task_data["behaviours"]
        self.curr_behaviour_id = task_data["current_behaviour_index"]
        self.status = task_data["status"]
        self.weight = task_data["weight"]

    def set_in_progress(self):
        self.status = self.STATUS_LIST["IN_PROGRESS"]

    def finished_behaviour(self):
        """
        Odpowiada za przypisanie kolejnego id zachowania do wykonania
        """
        if self.curr_behaviour_id == (len(self.behaviours)-1):
            # zakonczono zadanie
            self.status = self.STATUS_LIST["DONE"]
        else:
            # zakonczono zachowanie, ale nie zadanie
            self.curr_behaviour_id = self.curr_behaviour_id + 1

    def check_if_behaviour_is_go_to(self):
        behaviour = self.behaviours[self.curr_behaviour_id]
        is_go_to = "to" in behaviour["parameters"]
        return is_go_to


class Supervisor:
    def __init__(self, node_list, edge_list, tasks, robots_state_list):
        self.graph = gc.SupervisorGraphCreator(node_list, edge_list)
        self.tasks = []
        self.add_tasks(tasks)
        self.plan = {}
        self.update_robots_on_edge(robots_state_list)

    def run(self, robots_state_list):
        """
        Attributes:
            robots_state_list ([{"id": int, "isFree": bool, "taskId": int, "edge": (int,int),
                          "endBeh": bool, "planningOn": bool},...]) - stan robotow wychodzacy z symulatora
        """

        dispatcher = Dispatcher(self.graph, convert_robots_state_to_dispatcher_format(robots_state_list))
        self.update_robots_on_edge(robots_state_list)
        self.update_tasks_states(robots_state_list)
        robots = convert_robots_state_to_dispatcher_format(robots_state_list)
        self.plan = dispatcher.get_plan_all_free_robots(self.graph, robots, self.get_task_for_dispatcher())
        self.start_tasks()

    # oK
    def get_task_for_dispatcher(self):
        tasks = []
        for task in self.tasks:
            tasks.append({"id": task.id, "current_behaviour_index": task.curr_behaviour_id, "robot": task.robot_id,
                          "start_time": task.start_time, "status": task.status, "weight": task.weight,
                          "behaviours": task.behaviours})
        return tasks

    # OK
    def print_graph(self, plot_size=(45, 45)):
        graphData = self.graph.get_graph()
        plt.figure(figsize=plot_size)
        node_pos = nx.get_node_attributes(graphData, "pos")

        robots_on_edges_id = nx.get_edge_attributes(graphData, "robotsId")
        robots_on_edges = {}
        for edge in robots_on_edges_id:
            robots_on_edges[edge] = len(robots_on_edges_id[edge])
        node_col = [graphData.nodes[i]["color"] for i in graphData.nodes()]

        nx.draw_networkx(graphData, node_pos, node_color=node_col, node_size=3000, font_size=25,
                         with_labels=True, font_color="w", width=4)
        nx.draw_networkx_edge_labels(graphData, node_pos, node_color=node_col,
                                     edge_labels=robots_on_edges, font_size=30)
        plt.show()
        plt.close()

    # OK
    def print_graph_weights(self, plot_size=(45, 45)):
        graphData = self.graph.get_graph()
        plt.figure(figsize=plot_size)
        node_pos = nx.get_node_attributes(graphData, "pos")

        weights = nx.get_edge_attributes(graphData, "weight")
        node_col = [graphData.nodes[i]["color"] for i in graphData.nodes()]

        nx.draw_networkx(graphData, node_pos, node_color=node_col, node_size=3000, font_size=25,
                         with_labels=True, font_color="w", width=4)
        nx.draw_networkx_edge_labels(graphData, node_pos, node_color=node_col,
                                     edge_labels=weights, font_size=30)
        plt.show()
        plt.close()

    # OK
    def add_task(self, task_raw_data):
        """
        Parameters:
            task_raw_data ({"id": string, "behaviours": [{"id": int,  "parameters":{"name": Behaviour.TYPES[nazwa_typu],
                "to": "id_poi_string"},...], "robot": string, "start_time": "string_time",
                "current_behaviour_index": "string_id",  "weight": float, "status": Task.STATUS_LIST[nazwa_statusu]}):
                surowe dane o zadaniu
        """
        self.tasks.append(Task(task_raw_data))

    # OK
    def add_tasks(self, tasks_raw_data):
        """
        Parameters:
            tasks_raw_data ([{"id": string, "behaviours": [{"id": int,  "parameters":
                {"name": Behaviour.TYPES[nazwa_typu], "to": "id_poi_string"},...], "robot": string,
                "start_time": "string_time", "current_behaviour_index": "string_id",  "weight": float,
                 "status": Task.STATUS_LIST[nazwa_statusu]},...]): surowe dane o zadaniach
        """
        for task in tasks_raw_data:
            self.add_task(task)

    def get_task_by_id(self, task_id):
        given_task = None
        for task in self.tasks:
            if task.id == task_id:
                given_task = task
        return given_task

    # OK
    def update_tasks_states(self, robots_state_list):
        """
        Aktualizacja stanu zadan na podstawie danych otrzymanych z symulatora robotow, informuje o zakonczeniu
        zadan lub ich trwaniu

        Attributes:
            robots_state_list ([{"id": int, "isFree": bool, "taskId": int, "edge": (int,int),
                          "endBeh": bool, "planningOn": bool},...]) - stan robotow wychodzacy z symulatora
        """
        active_states = [state for state in robots_state_list if state["taskId"] is not None]
        for robotState in active_states:
            if robotState["planningOn"] and robotState["isFree"] and robotState["endBeh"]:
                for task in self.tasks:
                    if task.id == robotState["taskId"]:
                        task.finished_behaviour()

        # usuwanie ukonczonych zadan
        for task in self.tasks:
            if task.status == Task.STATUS_LIST["DONE"]:
                self.tasks.remove(task)

    # OK
    def start_tasks(self):
        """
        Ustawia status in progress dla przydzielonego zadania
        """
        for robot_id in self.plan:
            plan = self.plan[robot_id]
            for task in self.tasks:
                if plan["taskId"] == task.id and task.curr_behaviour_id == -1:
                    task.robot_id = robot_id
                    task.status = Task.STATUS_LIST["IN_PROGRESS"]
                    task.curr_behaviour_id = 0
                    break

    # OK
    def get_robots_command(self):
        """
        Zwraca liste komend przekazywanych do symulatora robotow.
        tasksList ([{'robotId': int, 'nextEdge': (int, int), 'taskId': int, 'endBeh': bool,
                     "taskDuration": int},...])
        """
        tasksList = []
        for i in self.plan.keys():
            robotPlan = copy.deepcopy(self.plan[i])
            robot_command = {"robotId": i, "nextEdge": robotPlan["nextEdge"],
                             "taskId": robotPlan["taskId"], "endBeh": robotPlan["endBeh"],
                             "taskDuration": self.graph.graph.edges[robotPlan["nextEdge"]]["weight"]}
            # robotCommand["taskDuration"] = 1
            tasksList.append(robot_command)
        return tasksList

    # OK
    def update_robots_on_edge(self, robots_state_list):
        """
        Parameters:
            robots_state_list ([{"id": int, "isFree": bool, "taskId": int, "edge": (int,int),
            "endBeh": bool, "planningOn": bool},...]) - stan listy robotow z symulatora
        """

        for edge in self.graph.graph.edges:
            self.graph.graph.edges[edge[0], edge[1]]["robotsId"] = [robot["id"] for robot in robots_state_list
                                                                    if robot["edge"] == edge]

    def print_data(self):
        print("\n---------------------------Stan zadan---------------------------")
        print("{:<8}{:<10}{:<20}{:<20}{:<10}{:<15}{}".format("taskId", "robotId", "WEIGHT", "start_time",
                                                             "status", "curr_beh_id", "behaviours"))
        for task in self.tasks:
            header = "{} {} {} ".format("id", "name", "goalId")
            data = []
            for behaviour in task.behaviours:
                if "to" in behaviour["parameters"]:
                    data.append("{:<83} {:<3} {:<6} {}".format("", str(behaviour["id"]),
                                                               str(behaviour["parameters"]["name"]),
                                                               str(behaviour["parameters"]["to"])))
                else:
                    data.append("{:<83} {:<3} {}".format("", str(behaviour["id"]),
                                                         str(behaviour["parameters"]["name"])))

            table = '\n'.join([header] + data)
            print("{:<11}{:<11}{:<7}{:<30}{:<15}{:<10}{}".format(str(task.id), str(task.robot_id), str(task.weight),
                                                                 str(task.start_time), str(task.status),
                                                                 str(task.curr_behaviour_id), table))

        print("\n---------------------------Nowy plan---------------------------")
        print("{:<9} {:<8} {:<12} {:<7} {:<5}".format('robotId', 'taskId', 'next edge', 'endBeh', 'taskDuration'))

        for plan in self.get_robots_command():
            print("{:<12} {:<6} {:<12} {:<10} {:<7}".format(str(plan['robotId']), str(plan["taskId"]),
                                                            str(plan["nextEdge"]),
                                                            str(plan["endBeh"]), str(plan["taskDuration"])))
