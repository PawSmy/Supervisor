import copy
import networkx as nx
import graph_creator as gc


class Task:
    """
    Attributes:
        data ("id": int, "robotId": int, "timeAdded": dataTime, "priority": Task.priority[...],
            "behaviours": [{"id": int, "status": Task.priority[...], "goalId": int, "name": Task.behType["goto"]}]): 
            dane o zadaniu z bazy
       }
    """
    priority = {  # slownik wartosci priorytetow
        "low": 1,  # zadanie malo istotne
        "normal": 2,  # normalne zadania generowane przez system
        "high": 3,  # dla zadan wymagajacych zjazdu do stacji ladowania
        "very_high": 4  # akcje zlecone przez serwisanta
    }

    beh_type = {  # slownik wartosci zachowan dla robota
        "goto": 1,
        "dock": 2,
        "wait": 3,
        "undock": 4
    }

    status = {  # slownik wartosci statusow zadania
        "not_started": 1,
        "in_progress": 2,
        "done": 3
    }

    def __init__(self, data):
        """
        Attributes:
            data ({"id": int, "behaviours": [{"id": int, "status": Task.status, "goalId": 
                                            int, "name":Task.behType["..."]},...],
                  "robotId": int, "timeAdded": time, "priority": Task.priority["..."]}): zadanie dla robota
        """
        self.data = data

    def get_first_undone_behaviour(self):
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

    def get_poi_goal(self):
        goalPoi = None
        previous_behaviour = self.data["behaviours"][0]
        for behaviour in self.data["behaviours"]:
            if "goalId" in behaviour:
                if goalPoi is None or behaviour["status"] == self.status["done"]:
                    # własciwy dojazd do POI został wykonany, ale kolejnym zachowaniem może być
                    # dokowanie,wait,oddokowanie
                    goalPoi = behaviour["goalId"]
                if previous_behaviour["status"] == "done" and behaviour["status"] != self.status["done"]:
                    # poprzednie zachowanie zostało zakończone, aktualne posiada jakiś inny status
                    # dla tego nowego zachowania robot będzie jechał do POI przez nie wskazanego
                    goalPoi = behaviour["goalId"]
            previous_behaviour = behaviour
        return goalPoi

    def get_current_behaviour(self):
        for behaviour in self.data["behaviours"]:
            if behaviour["status"] != self.status["done"]:
                return behaviour

    def get_task_first_goal(self):
        """Dla zadania zwracane jest pierwsze POI do ktorego ma dojechac robot w ramach podanego zadania.
        Returns:
            goalId (int): id POI z bazy
        """
        for behaviour in self.data["behaviours"]:
            if "goalId" in behaviour:
                return behaviour["goalId"]


class Dispatcher:
    """
    Klasa dispatchera odpowiadajaca za tworzenie planu, przydzielanie zadan do robotow i unikanie drog
    kolizyjnych na grafie. Wolne roboty blokujace stanowiska kierowane sa odpowiednio na miejsca oczekiwan
    queue/parking.

    Attributes:
        pois ():
        graph (nx.DiGraph()): wygenerwany rozszerzony graf do planowania kolejnych zachowan robotow
        all_tasks ([{"id": int, "behaviours": BehavioursList, "robotId": int, "timeAdded": dataTime, "priority": int}
            ,...]): Zawiera liste aktualnie wykonywanych i nowych zadan. Sa one przypisywane do zadan w
            robots_tasks, a nastepnie usuwane z tej listy.
        robots_tasks ([{"id": (int), "edge": (int,int), "planningOn": bool, "isFree": bool, "task": Task,
                        "nextTaskEdge": (int,int), "endBehEdge": bool},...]): Zawiera liste aktywnych robotow w
                        systemie wraz z przydzielonymi
                        do nich zadaniami.
            "id" - id robota
            "edge" - aktualna krawedz na ktorej znajduje sie robot, jesli jest free to znajduje sie w wezle koncowym
            "planningOn" - robot w trybie autonomicznym, ma dostawac zadania z dispatchera
            "isFree" - robot ukonczyl fragment zachowania i oczekuje na przydzial kolejnego fragmentu zachowania
            "task" - przypisane zadanie do robota (aktualne lub nowe)
            "nextTaskEdge" - aktualna krawedz na ktorej robot wykonuje zadanie lub gdy robot jest wolny to
                            przypisana jest krawedz grafu wedlug ktorej ma wykonac kolejna akcje o ile ta krawedz
                            jest wolna. Jesli jest zajeta to zadanie jest przypisane, ale nie ma krawedzi ruchu.
            "endBehEdge" - okresla czy dana krawedz przejscia konczy dane zachowanie czy nie
    """

    def __init__(self, graph_data, poi_list, robots):
        """Utworzenie klasy Dispatcher'a i ustawienie wartosci atrybutow pois, graph, all_tasks.

        Parameters:
            graph_data (SupervisorGraphCreator): wygenerwany rozszerzony graf do planowania kolejnych zachowan robotow
            poi_list ({poiId: {"type": nodeType}, ...}): zawiera liste poi wraz z Id oraz jego typem
            robots ():
        """
        self.graph = copy.deepcopy(graph_data.get_graph())
        self.extend_graph()
        self.robots_tasks = []
        self.set_robots(copy.deepcopy(robots))
        self.pois = []
        self.set_pois(copy.deepcopy(poi_list))
        self.all_tasks = []

    def extend_graph(self):
        for edge in self.graph.edges(data=True):
            self.graph.edges[edge[0], edge[1]]["robotsId"] = []
            self.graph.edges[edge[0], edge[1]]["planWeight"] = self.graph.edges[edge[0], edge[1]]["weight"]

    def block_other_pois(self, robot_node, target_node):
        """
        Funkcja odpowiada za zablokowanie krawedzi grafu zwiazanych z innymi POI niz aktualnym (jesli jest w POI) i
        docelowym.

        Parameters:
            robot_node (int): wezel grafu z supervisora w ktorym aktualnie jest robot
            target_node (int): wezel grafu z supervisora do ktorego zmierza robot
        """
        no_block_poi_ids = [0, self.graph.nodes[robot_node]["poiId"], self.graph.nodes[target_node]["poiId"]]
        for edge in self.graph.edges(data=True):
            start_node_poi_id = self.graph.nodes[edge[0]]["poiId"]
            end_node_poi_id = self.graph.nodes[edge[1]]["poiId"]
            if start_node_poi_id in no_block_poi_ids and end_node_poi_id in no_block_poi_ids:
                self.graph.edges[edge[0], edge[1]]["planWeight"] = self.graph.edges[edge[0], edge[1]]["weight"]
            else:
                self.graph.edges[edge[0], edge[1]]["planWeight"] = None

    def set_plan(self, robots, tasks):
        """Zwraca plan dla robotow.
        Parameters:
            robots ():
            tasks ():
        """
        self.set_tasks(tasks)
        self.set_robots(robots)
        self.set_tasks_doing_by_robots()
        self.set_task_assigned_to_robots()
        self.set_other_tasks()

    def get_plan_all_free_robots(self, graph_data, robots, tasks):
        self.graph = graph_data.get_graph()
        self.set_plan(robots, tasks)
        plan = []
        for r_task in self.robots_tasks:
            if r_task["nextTaskEdge"] is not None:
                plan.append({"robotId": r_task["id"], "taskId": r_task["task"]["id"],
                             "nextEdge": r_task["nextTaskEdge"], "endBeh": r_task["endBehEdge"]})
        return plan

    def get_plan_selected_robot(self, graph_data, robots, tasks, robot_id):
        self.graph = graph_data.get_graph()
        self.set_plan(robots, tasks)
        given_robot = [rTask for rTask in self.robots_tasks if rTask["id"] == robot_id][0]
        if given_robot["nextTaskEdge"] is None:
            return None
        else:
            return {"robotId": given_robot["id"], "taskId": given_robot["task"]["id"],
                    "nextEdge": given_robot["nextTaskEdge"], "endBeh": given_robot["endBehEdge"]}

    def set_robots(self, robots):
        """Ustawia roboty aktywnie dzialajace w systemie tzn. podlegajace planowaniu i przydzielaniu zadan.
        Dokonuje konwersji danych wejsciowych do formatu obslugiwanego przez dispatchera i zapisanych w
        atrybucie: robots_tasks

        Parameters:
            robots ([ {"id": int, "edge": (int,int), "planningOn": bool, "isFree": bool},. ..]): lista robotow
                        przekazana przez supervisora
                    id - robota,
                    edge - krawedz grafu na ktorej znajduje sie robot,
                    planningOn - robot w trybie autonomicznym, ma miec przydzielane zadania, aktywny w systemie
                    isFree - status zglaszany przez robota informujacy o zakonczeniu wykonywania zleconego zachowania

        TODO
            - przekonwertowac dane z supervisora do odpowiedniego formatu zgodnego z atrybutem robots_tasks
        """
        robots = copy.deepcopy(robots)
        self.robots_tasks = []
        for robot in robots:
            if robot["planningOn"]:
                robot["task"] = None
                robot["nextTaskEdge"] = None
                robot["endBehEdge"] = None
                self.robots_tasks.append(robot)

    def set_tasks(self, tasks):
        """Ustawia zadania, ktore sa aktualnie wykonywane lub do wykonania w systemie. Beda one pobierane
        z listy atrybutow all_tasks i przypisywane do robotow. Struktura zadan powinna byc zgodna ze struktura
        wystepujaca w bazie. Dodatkowo sortuje zadanie po priorytecie i czasie wprowadzenia do systemu od
        najstarszego.

        Parameters:
            tasks ([Task,Task]): lista poi przekazana przez supervisora na podstawie danych w bazie. Zawiera aktualnie
                wykonywane lub do wykonania zadania. Lista powinna zawierac zadania wykonywane lub do wykonania.
        """
        max_priority_value = 0
        all_tasks = copy.deepcopy(tasks)
        for task in all_tasks:
            max_priority_value = task["priority"] if task["priority"] > max_priority_value else max_priority_value
        # odwrocenie wynika pozniej z funkcji sortujacej, zadanie o najwyzszym priorytecie powinno miec
        # najnizsza wartosc liczbowa
        for task in all_tasks:
            task["priority"] = max_priority_value - task["priority"]

        # sortowanie zadan po priorytetach i czasie zgłoszenia
        tasks_id = [data["id"] for data in sorted(all_tasks, key=lambda task_data: (task_data["priority"],
                                                                                    task_data["timeAdded"]),
                                                  reverse=False)]
        sorted_tasks = []
        for i in tasks_id:
            # wybieranie po kolei id z listy zadan i dodawanie ich do listy
            task = [task for task in all_tasks if task["id"] == i][0]
            sorted_tasks.append(task)
        self.all_tasks = sorted_tasks

    def set_pois(self, poi_data):
        """Ustawia wszystkie dostepne POI w systemie.

        Parameters
            poi_data (): dane o POI wraz z ID i typem
        """
        self.pois = copy.deepcopy(poi_data)

    def set_tasks_doing_by_robots(self):
        """Przypisanie zadan do robotow, ktore sa aktualnie wykonywane.
        """
        tasks_to_remove = []
        for in_task in self.all_tasks:
            for r_task in self.robots_tasks:
                if r_task["id"] == in_task["robotId"] and \
                        in_task["behaviours"][0]["status"] != Task.status["not_started"]:
                    r_task["task"] = in_task
                    self.set_task_edge(r_task)
                    tasks_to_remove.append(in_task)
        for rm_task in tasks_to_remove:
            self.all_tasks.remove(rm_task)

    def set_task_assigned_to_robots(self):
        """Przypisanie do wolnych robotow zadan, ktore sa do nich przypisane. Po przypisaniu zadania
        usuwane jest ono z listy all_tasks.
        """
        tasks_to_remove = []
        for in_task in self.all_tasks:
            for r_task in self.robots_tasks:
                if r_task["task"] is None:
                    if in_task["robotId"] == r_task["id"] and \
                            in_task["behaviours"][0]["status"] == Task.status["not_started"]:
                        r_task["task"] = in_task
                        self.set_task_edge(r_task)
                        tasks_to_remove.append(in_task)
        for rm_task in tasks_to_remove:
            self.all_tasks.remove(rm_task)

    def get_free_robots(self):
        """Zwraca liste robotow do ktorych zadania nie zostaly przypisane

        Returns:
            list ([{"id": (int), "edge": (int,int), "planningOn": bool, "task": Task},...]): lista robotow,
                ktore nie posiadaja przypisanych zadan. Pole 'task' wynosi None.
        """
        return [robot for robot in self.robots_tasks if robot["planningOn"] and robot["task"] is None]

    def set_other_tasks(self):
        """Przypisanie zadan do pozostalych robotow z uwzglednieniem pierwszenstwa przydzialu zadan do robotow
        blokujacych POI.
        """
        while True:
            free_robots_id = [robot["id"] for robot in self.get_free_robots()]
            blocking_robots_id = self.get_robots_blocking_poi()

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

    def set_task_edge(self, robot_task):
        """
        Ustawia kolejna krawedz przejscia dla wolnych robotow.
            robot_task (): informacje o robocie i przydzielonym do niego zadaniu
        """
        if robot_task["planningOn"] and robot_task["isFree"]:
            # robot wykonal fragment zachowania lub ma przydzielone zupelnie nowe zadanie
            # weryfikacja czy kolejna akcja jazdy krawedzia moze zostac wykonana
            # 1. pobranie kolejnej krawedzi ktora ma poruszac sie robot
            end_node = self.get_undone_behaviour_node(robot_task["task"])
            assert robot_task["edge"][
                       1] != end_node, "Robot finished behaviour in task, but behaviour doesn't have " \
                                       "done status or invalid node found."
            start_node = robot_task["edge"][1]
            self.block_other_pois(start_node, end_node)
            path_nodes = nx.shortest_path(self.graph, source=start_node, target=end_node,
                                          weight='planWeight')
            next_edge = (path_nodes[0], path_nodes[1])
            # 2. sprawdzenie czy krawedz nalezy do grupy krawedzi na ktorej moze byc tylko 1 robot
            group_id = self.graph.edges[next_edge]["edgeGroupId"]
            if group_id != 0:
                # krawedz nalezy do grupy
                n_robots_in_group = sum([len(edge[2]["robotsId"]) for edge in self.graph.edges(data=True)
                                         if edge[2]["edgeGroupId"] == group_id])
                robots_next_edges = [robot for robot in self.robots_tasks
                                     if robot["isFree"] and robot["nextTaskEdge"] is not None]
                n_robots_in_next_group = sum([1 for robot in robots_next_edges
                                              if self.graph.edges[robot["nextTaskEdge"]]["edgeGroupId"] == group_id])
                # sprawdzenie czy dany robot znajduje sie w tej grupie, jesli tak to nie jest wliczany
                robots_ids_in_group = [edge[2]["robotsId"] for edge in self.graph.edges(data=True)
                                       if edge[2]["edgeGroupId"] == group_id]
                remove_robot = 1 if any(robot_task["id"] in r_ids for r_ids in robots_ids_in_group) else 0

                edge_is_available = 1 > (n_robots_in_group + n_robots_in_next_group - remove_robot)
            else:
                # krawedz nie nalezy do grupy
                # 3. sprawdzenie ile aktualnie robotow porusza sie dana krawedzia
                n_robots_on_edge = len(self.graph.edges[next_edge]["robotsId"])
                # 4. sprawdzenie ile robotow ze statusem isFree ma juz przypisana ta krawedz
                n_robots_on_next_edge = sum([1 for robot in self.robots_tasks
                                             if robot["isFree"] and robot["nextTaskEdge"] == next_edge])
                # 5. pobranie maksymalnej liczby robotow i sprawdzenie czy ta liczba jest wieksza niz suma robotow
                # z punktu 2 i 3. jesli liczba jest mniejsza to ta krawedz jest przypisana
                max_robots = self.graph.edges[next_edge]["maxRobots"]
                edge_is_available = max_robots > (n_robots_on_edge + n_robots_on_next_edge)
            if edge_is_available:
                # jesli krawedz konczy dane zachowanie to ustawiany jest parametr endBehEdge na True
                behaviour = Task(robot_task["task"]).get_first_undone_behaviour()
                if behaviour["name"] != Task.beh_type["goto"]:  # dock,wait,undock -> pojedyncze przejscie na grafie
                    robot_task["endBehEdge"] = True
                elif len(path_nodes) == 2:
                    robot_task["endBehEdge"] = True
                else:
                    robot_task["endBehEdge"] = False
                robot_task["nextTaskEdge"] = next_edge
            else:
                robot_task["nextTaskEdge"] = None
        else:
            robot_task["nextTaskEdge"] = None

    def get_robots_blocking_poi(self):
        """Zwraca liste robotow, ktore dla aktualnego przydzialu zadan sa zablokowane przez roboty bez zadan.

        Returns:
            list ([robotId,...]): lista zawierajaca ID robotow, ktore blokuja POI.
        """
        # dotyczy tylko robotów, które nie mają zadań
        poi_robots = []
        for robot in self.robots_tasks:
            if robot["task"] is None:
                # pobranie aktualnego POI w którym jest robot
                graph_node_id = robot["edge"][1]
                # print(graphNodeId)
                poi_id = self.graph.nodes[graph_node_id]["poiId"]
                # jesli poi jest miejscem kolejkowania robotow to jest pomijane, taki robot nie blokuje POI
                # do POI moze byc skierowany dodatkowy robot pod warunkiem, ze zmiesci sie na krawedzi oczekiwania
                # wynika z warunkow maksymalnej liczby kolejkowania robotow przed POI
                if poi_id != 0:
                    if self.pois[poi_id]["type"] != gc.base_node_type["queue"]:
                        # roboty bez zadań, które aktualnie są w POI stanowiskowym lub na parkingu
                        poi_robots.append({"robotId": robot["id"], "poiId": poi_id})
        # sprawdzenie czy roboty, które aktualnie wykonują swoje zachowanie jadą do tego POI
        busy_pois = [poi["poiId"] for poi in poi_robots]
        busy_pois = list(dict.fromkeys(busy_pois))
        blocked_pois = []
        for robot in self.robots_tasks:
            if robot["task"] is not None:
                poi_id = self.get_current_destination_goal(robot["id"])
                if poi_id in busy_pois and poi_id not in blocked_pois:
                    blocked_pois.append(poi_id)
        # zwracana jest lista robotów, które blokują POI do których aktualnie jadą roboty
        return [robot["robotId"] for robot in poi_robots if robot["poiId"] in blocked_pois]

    def get_current_destination_goal(self, robot_id):
        """Zwraca Id POI do ktorego aktualnie jedzie robot lub wykonuje w nim jakas operacje (dokowanie,
            wait, oddokowanie)

        Parameters:
            robot_id (int): id robota dla ktorego sprawdzany jest aktualny cel do ktorego jedzie.

        Returns:
            goalPoi (int): id POI z bazy do ktorego aktualnie jedzie robot, jesli nie jest on skierowany do
                zadnego POI to zwracana jest wartosc None
        """
        robot = [robot for robot in self.robots_tasks if robot["id"] == robot_id][0]
        if robot["task"] is not None:
            task = Task(robot["task"])
            return task.get_poi_goal()
        else:
            # robot nie ma przydzielonego zadania
            return None

    def get_robots_using_pois(self):
        """Zwraca liczbe robotow, ktore aktualnie uzywaja danego POI. Przez uzywanie POI rozumie sie
        dojazd do POI, dokowanie, obsluge w POI lub oddokowanie.

        Returns:
            robotsToPoi ({poiId: int, ...}): Slownik z liczba robotow dla ktorego kluczem jest ID POI z bazy
                a wartoscia liczba robotow zmierzajaca/bedaca do danego POI
        """
        robots_to_poi = {i: 0 for i in self.pois.keys()}
        for robot in self.robots_tasks:
            goal_id = self.get_current_destination_goal(robot["id"])
            if goal_id is not None:
                robots_to_poi[goal_id] = robots_to_poi[goal_id] + 1
        return robots_to_poi

    @property
    def get_max_allowed_robots_using_pois(self):
        """Zwraca liste zawierajaca maksymalna liczbe robotow, ktora moze byc obslugiwana przez dany typ POI.
        Przekroczenie tej liczby oznacza, ze roboty moga zaczac sie kolejkowac na glownym szlaku komunikacyjnym.

        Returns:
            maxRobotPois ({poiId: int, poiId2: int,...}): Slownik z liczba robotow dla ktorego kluczem
                jest ID POI z bazy a maksymalna liczba robotow jaka moze oczekiwac i byc obslugiwana
                przy stanowisku.
        """
        max_robot_pois = {i: 0 for i in self.pois.keys()}
        connected_edges = [edge for edge in self.graph.edges(data=True) if "connectedPoi" in edge[2]]

        for edge in connected_edges:
            poiId = edge[2]["connectedPoi"]
            poi = self.pois[poiId]
            if poi["type"] == gc.base_node_type["parking"]:
                # dla POI parkingowych tylko 1 robot
                max_robot_pois[poiId] = 1
            elif poi["type"] == gc.base_node_type["waiting"]:
                # dla POI z kolejkowaniem tylko tyle robotów ile wynika z krawędzi oczekiwania
                max_robot_pois[poiId] = edge[2]["maxRobots"]
                pass
            else:
                # 1 dla obsługi samego stanowiska + maksymalna liczba robotów na krawędzi związana z danym POI
                max_robot_pois[poiId] = edge[2]["maxRobots"] + 1
        return max_robot_pois

    def get_free_task_to_assign(self, robots_numbers):
        """Zwraca liste zadan, ktore nalezy przypisac do robotow. Jako argument podawana jest liczba robotow
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
        max_robots_using_poi = self.get_robots_using_pois()
        max_robots_in_poi = self.get_max_allowed_robots_using_pois
        free_slots_in_poi = {}
        for poi_id in max_robots_using_poi:
            diff = max_robots_in_poi[poi_id] - max_robots_using_poi[poi_id]
            free_slots_in_poi[poi_id] = 0 if diff < 0 else diff
        # Lista zadań, które nie są przypisane do robotów
        free_tasks = []
        all_free_tasks = [task for task in self.all_tasks if task["robotId"] == 0]
        for task_data in all_free_tasks:
            task = Task(task_data)
            goal_id = task.get_task_first_goal()
            if free_slots_in_poi[goal_id] > 0:
                free_slots_in_poi[goal_id] = free_slots_in_poi[goal_id] - 1
                free_tasks.append(task_data)
                if len(free_tasks) == robots_numbers:
                    break
        return free_tasks

    def assign_tasks_to_robots(self, tasks, robots_id):
        """Optymalne przypisanie zadan do robotow pod katem wykonania. W pierwszej kolejnosci zadanie o najwyzszym
        priorytecie dostaje robot, ktory wykona je najszybciej. Po przypisaniu zadania usuwane jest ono z listy
        all_tasks.

        Parameters:
            tasks ([Task, ...]): lista zadan, ktore maja byc przypisane do robotow
            robots_id ([int, int, ...]): lista id robotow do ktorych maja zostac przypisane zadania

        TODO
            - optymalne przypisanie robotow do zadan.
        """
        for task in tasks:
            fastest_robot_id = None
            min_task_time = None
            for r_id in robots_id:
                robot_node = [r_task["edge"][1] for r_task in self.robots_tasks if r_task["id"] == r_id][0]
                target_node = self.get_undone_behaviour_node(task)
                self.block_other_pois(robot_node, target_node)
                task_time = nx.shortest_path_length(self.graph, source=robot_node, target=target_node,
                                                    weight='planWeight')
                if min_task_time is None or min_task_time > task_time:
                    fastest_robot_id = r_id
                    min_task_time = task_time
            # Przypisanie robota do zadania
            for r_task in self.robots_tasks:
                if r_task["id"] == fastest_robot_id:
                    r_task["task"] = task
                    self.set_task_edge(r_task)
                    break
            robots_id.remove(fastest_robot_id)
            self.all_tasks.remove(task)

    def send_free_robots_to_parking(self, blocking_robots_id):
        """Tworzy i przypisuje do robotow blokujacych POI zadanie jazdy do Parkingu/Queue w celu odblokowania
        POI.

        Parameters:
            blocking_robots_id ([int,int,...]): lista id robotow, ktore blokuja POI

        TODO
            - utworzenie zadan dojazdu w wolne miejsce dla robotow blokujacych POI
            - przypisanie zadan do robotow
        """
        pass

    def send_busy_robots_to_parking(self, blocking_robots_id):
        """Tworzy i przypisuje do robotow blokujacych POI zadanie jazdy do Parkingu/Queue w celu odblokowania
        POI.

        Parameters:
            blocking_robots_id ([int,int,...]): lista id robotow, ktore blokuja POI

        TODO
            - przypisanie zadan do robotow
            - obsluga robotow ktore wykonuja inne zadanie i musza zjechac na parking
        """
        pass

    def get_undone_behaviour_node(self, task_data):
        """
        Wybiera z zadania pierwsze zachowanie, ktore nie zostalo jeszcze ukonczone, dla zadan nie rozpoczetych
        jest to pierwsze zachowanie jakie wystepuje w zadaniu.

        Parameters:
            task_data (Task): zadanie dla ktorego okreslany jest wezel do ktorego ma dotrzec robot w zaleznosci
                od kolejnego zadania do wykonania

        Returns:
            goalNode (int): id wezla z grafu rozszerzonego na ktorym opiera sie tworzenie zadan
        """
        task = Task(task_data)
        poi_id = task.get_poi_goal()
        behaviour = task.get_current_behaviour()
        goalNode = None
        if behaviour["name"] == Task.beh_type["goto"]:
            goalNode = self.get_end_go_to_node(poi_id)
        elif behaviour["name"] == Task.beh_type["dock"]:
            goalNode = self.get_end_docking_node(poi_id)
        elif behaviour["name"] == Task.beh_type["wait"]:
            goalNode = self.get_end_wait_node(poi_id)
        elif behaviour["name"] == Task.beh_type["undock"]:
            goalNode = self.get_end_undocking_node(poi_id)
        return goalNode

    def get_end_go_to_node(self, poi_id):
        """
        Zwraca węzeł końcowy krawędzi dla zadania typu GOTO POI.

        Parameters:
            poi_id (int): id POI z bazy

        Returns:
            graphNode (int): koncowy wezel krawedzi dojazdu do wlasciwego stanowiska
        """
        pois_nodes = [node for node in self.graph.nodes(data=True) if "poiId" in node[1]]
        poi_type = self.pois[poi_id]["type"]["nodeSection"]
        if poi_type == gc.base_node_section_type["dockWaitUndock"]:
            return [node[0] for node in pois_nodes if node[1]["poiId"] == poi_id
                    and node[1]["nodeType"] == gc.new_node_type["dock"]][0]
        elif poi_type == gc.base_node_section_type["waitPOI"]:
            return [node[0] for node in pois_nodes if node[1]["poiId"] == poi_id
                    and node[1]["nodeType"] == gc.new_node_type["wait"]][0]
        else:
            return [node[0] for node in pois_nodes if node[1]["poiId"] == poi_id][0]

    def get_end_docking_node(self, poi_id):
        """
        Zwraca węzeł końcowy krawędzi dla zadania typu DOCK POI.

        Parameters:
            poi_id (int): id POI z bazy

        Returns:
            graphNode (int): koncowy wezel krawedzi zwiazanej z zachowaniem dokowania
        """
        pois_nodes = [node for node in self.graph.nodes(data=True) if "poiId" in node[1]]
        poi_node = [node for node in pois_nodes if node[1]["poiId"] == poi_id
                    and node[1]["nodeType"] == gc.new_node_type["wait"]]
        return poi_node[0][0]

    def get_end_wait_node(self, poi_id):
        """
        Zwraca węzeł końcowy krawędzi dla zadania typu WAIT POI.

        Parameters:
            poi_id (int): id POI z bazy

        Returns:
            graphNode (int): koncowy wezel krawedzi zwiazanej z zachowaniem WAIT
        """
        pois_nodes = [node for node in self.graph.nodes(data=True) if "poiId" in node[1]]
        poi_type = self.pois[poi_id]["type"]["nodeSection"]
        poi_node = None
        if poi_type == gc.base_node_section_type["dockWaitUndock"]:
            poi_node = [node for node in pois_nodes if node[1]["poiId"] == poi_id
                        and node[1]["nodeType"] == gc.new_node_type["undock"]]
        elif poi_type == gc.base_node_section_type["waitPOI"]:
            poi_node = [node for node in pois_nodes if node[1]["poiId"] == poi_id
                        and node[1]["nodeType"] == gc.new_node_type["end"]]
        return poi_node[0][0]

    def get_end_undocking_node(self, poi_id):
        """
        Zwraca węzeł końcowy krawędzi dla zadania typu UNDOCK POI.

        Parameters:
            poi_id (int): id POI z bazy

        Returns:
            graphNode (int): koncowy wezel krawedzi zwiazanej z zachowaniem UNDOCK
        """
        pois_nodes = [node for node in self.graph.nodes(data=True) if "poiId" in node[1]]
        poi_node = [node for node in pois_nodes if node[1]["poiId"] == poi_id
                    and node[1]["nodeType"] == gc.new_node_type["end"]]

        return poi_node[0][0]
