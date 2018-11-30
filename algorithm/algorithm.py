import copy

class RearrangableDynamicSteinerTree(object):

    def __init__(self, num_nodes, edge_costs, num_requests, change_requests,
                 terminals, initial_node, delta):
        """ Initialize a EBA Algorithm object

        Args:
             num_nodes: an integer denoting the maximum number of nodes that
                will ever be added to the steiner tree. This is assumed to be
                known ahead of any computation.
            edge_costs: an NxN matrix of floats (where N == num_nodes) where
                edge_costs[i][j] == edge_cost[j][i] == the cost of connecting
                nodes i and j
            num_requests: the number of change requests
            change_requests: an array of tuples (i, "add") or (i, "remove")
                where i is the index of a node that is not yet in the tree
                at that time
            terminals: a boolean list of size num_nodes in which terminal[i] ==
                True if i is in the terminal set
            initial_node: the index of a initial node which the computation
                begins with. It is assumed that this node is in the terminal
                set.
            delta: delta. fucking obviously
         """

        self.num_nodes = num_nodes
        self.cost = edge_costs
        self.is_terminal = terminals
        self.num_requests = num_requests
        self.change_requests = change_requests

        # initially there is a single node in the tree
        self.tree_nodes_t = [[initial_node]]
        self.terminals_t = [[initial_node]]

        # a list of adjacency matrix for each time i
        # after run_eba_algorithm is called, this will contain num_requests + 1
        # adjacency matrices, one for each step of the algorithm
        self.edge_matrix_t = [ [[0 for i in range(self.num_nodes)] for j in
                                range(self.num_nodes)] ]

    def get_edge_matrix_at_time(self, time):
        return self.edge_matrix_t[time]

    def run_eba_algorithm(self):
        """ Runs the Edge Bounded algorithm outlined in the paper Dynamic
         Steiner Tree Problem. Imase & Waxman (1989) """
        i = 0

        for i in range(1, self.num_requests+1):
            # Si-1
            print(i)
            vertice_set = copy.deepcopy(self.tree_nodes_t[i-1])
            terminal_set = copy.deepcopy(self.terminals_t[i-1])
            edge_matrix = copy.deepcopy(self.edge_matrix_t[i-1])

            ri, command = self.change_requests[i-1]
            if command == "add":
                # Si = Si-1 U {ri}
                if self.is_terminal[ri]:
                    if ri not in terminal_set:
                        terminal_set.append(ri)

                if ri not in vertice_set:
                    vertice_set.append(ri)

                # add(ri, T, S)
                self._add_node(ri, vertice_set, terminal_set, edge_matrix)

            elif command == "remove":
                # Si = Si-1 - {ri}
                if self.is_terminal[ri]:
                    if ri in terminal_set:
                        terminal_set.remove(ri)
                # if ri in vertice_set:
                #     vertice_set.remove(ri)
                # remove(T, S)
                self._remove_node(vertice_set, terminal_set, edge_matrix)
            else:
                raise ValueError("unrecognized command {0}".format(command))

            self.tree_nodes_t.append(vertice_set)
            self.terminals_t.append(terminal_set)
            self.edge_matrix_t.append(edge_matrix)

    def _add_node(self, node, vertice_set, terminal_set, edge_matrix):
        """ Add subroutine

        Args:
            node: an index denoting a node to add
            vertice_set: a reference to a list fo nodes in the tree
            terminal_set: a reference to a list of terminal nodes in the tree
            edge_matrix: a reference to an adjacency matrix
        """
        # TODO: write add subroutine
        # get the edges between v and nodes in the vertice_set call it W
        edges = []
        for i in range(len(edge_matrix[node])):
            if i == node:
                continue
            if i in vertice_set:
                edges += [(node, i)]
        # sort the edges
        edges = sorted(edges, key=lambda p: self.cost[p[0]][p[1]])
        # select the minimum cost edge (v, w1)
        # subtract (v, w1) from W
        _, w1 = edges.pop(0)
        # add (v, w1) to edge_matrix
        self.add_edge(node, w1, edge_matrix)
        # add v to vertice_set
        vertice_set += [w1]
        # while W is not empty
        while len(edges) > 0:
            # find the minimum edge in W (v, w1)
            _, w1 = edges.pop(0)
            # remove (v, w1) from W
            # find the single path connecting (v, w1) i.e p(v, w1; T);
            path = self.get_path_connecting(node, w1, edge_matrix)
            # find the maximum edge in that path e
            max_edge, max_cost = self.max_edge_in_path(path)

            # if cost(e) > delta * cost(v, w1) ->
            if max_cost > delta * self.cost[node][w1]:
                # T := T - e + (v, w1)
                # remove e from edge_matrix
                self.remove_edge(max_edge[0], max_edge[1], edge_matrix)
                # add (v, w1) from edge_matrix
                self.add_edge(node, w1, edge_matrix)

        self._remove_node(vertice_set, terminal_set, edge_matrix)

    def _remove_node(self, vertice_set, terminal_set, edge_matrix):
        """ Remove node subroutine

        Args:
            vertice_set: reference to a list of nodes in the tree
            terminal_set: reference to a list of terminal nodes in the tree
            edge_matrix: reference to a adjacency matrix
         """
        # TODO: write remove subroutine
        # let W be the set of vertices_set - terminal_set
        non_terminals = [node for node in vertice_set if node not in terminal_set]
        # for each node in W
        for node in non_terminals:
            if self.degree(node, edge_matrix) == 1:
                # remove node
                x1 = self.get_neighbors(node, edge_matrix)[0]
                self.remove_edge(node, x1, edge_matrix)
                pass
            if self.degree(node, edge_matrix) == 2:
                # x1 and x2 are nodes adjacent to node
                neighbors = self.get_neighbors(node, edge_matrix)
                x1, x2 = neighbors[0], neighbors[1]
                # for each u1_node in x1 connected component
                x1_comp = self.connected_component(x1, edge_matrix)
                x2_comp  = self.connect_component(x2, edge_matrix)
                min_edge = None
                min_cost = float("inf")
                for u1_node in x1_comp:
                    # for each u2_node in x2 connected component
                    for u2_node in x2_comp:
                        # find the min g(u1_node, u2_node)
                        # g = max edge cost on path from u1_node to u2_node
                        # path = self.get_path_connecting(u1_node, u2_node,
                        #                                 edge_matrix)
                        cost = self.cost[u1_node][u2_node]
                        # edge, cost = self.max_edge_in_path(path, edge_matrix)
                        if cost > min_cost:
                            min_cost = cost
                            min_edge = (u1_node, u2_node)
                self.remove_edge(node, x1, edge_matrix)
                self.remove_edge(node, x2, edge_matrix)
                self.add_edge(min_edge[0], min_edge[1])

    def add_edge(self, u, v, edge_matrix):
        edge_matrix[u][v] = 1
        edge_matrix[v][u] = 1

    def remove_edge(self, u, v, edge_matrix):
        edge_matrix[u][v] = 0
        edge_matrix[v][u] = 0

    def get_neighbors(self, node, edge_matrix):
        return [i for i in range(len(edge_matrix[node]))
                if edge_matrix[node][i] == 1]

    def get_path_connecting(self, u, v, edge_matrix):
        """ Find a path to v from u in a tree via depth first search


        Returns:
             a list of ordered pairs denoting edges from u to v
         """
        closed = set()
        parent = [-1 for i in range(len(edge_matrix))]
        frontier = [u]
        found_v = False
        while len(frontier) > 0:
            s = frontier.pop()
            closed.add(s)

            if s == v:
                found_v = True
                break

            for node in self.get_neighbors(s, edge_matrix):
                if node not in closed and node not in frontier:
                    parent[node] = s
                    frontier.insert(0, node)

        if not found_v:
            # maybe print debug info
            assert False, "You searched for a path connecting two nodes which " \
                          "were not connected. you should feel much shame"

        path = []
        s = v
        while s != u:
            prev = s
            s = parent[s]
            path.insert(0, (prev, s))
        return path

    def connected_component(self, node, edge_matrix):
        """ find the connected component node is in

        Returns:
            a list of vertices in the connected component of node
        """
        closed = set()
        frontier = [node]

        while len(frontier):
            s = frontier.pop()
            closed.add(s)

            for node in self.get_neighbors(s, edge_matrix):
                if node not in closed and node not in frontier:
                    parent[node] = s
                    frontier.insert(0, node)
        return list(closed)

    def max_edge_in_path(self, path):
        """ find the max cost edge in a path """
        max_edge = path[0]
        max_cost = self.cost[max_edge[0]][max_edge[1]]
        for i in range(1, len(path)):
            cost = self.cost[path[i][0]][path[i][1]]
            if cost > max_cost:
                max_cost = cost
                max_edge = path[i]
        return max_edge, max_cost

    def degree(self, node, edge_matrix):
        """ Calculated the degree of a node in a graph defined by an adjacency
        matrix
        """
        count = 0
        for i in range(len(edge_matrix[node])):
            if edge_matrix[node][i] == 1:
                count += 1
        return count

def index_of(i, l):
    ret_index = None
    for index in range(len(l)):
        if l[index] == i:
            ret_index = index
    return ret_index

if __name__ == "__main__":
    num_nodes = 3
    edge_costs = [[0, 1, 1], [1, 0, 1], [1, 1, 0]]
    num_requests = 3
    change_requests = [[1, "add"], [2, "add"], [1, "remove"]]
    terminals = [1, 0, 1]
    initial_node = 0
    delta = 1.5

    l = RearrangableDynamicSteinerTree(num_nodes, edge_costs, num_requests,
                                       change_requests, terminals, initial_node,
                                       delta)
    l.get_edge_matrix_at_time(0)
    l.run_eba_algorithm()

    print(l.get_edge_matrix_at_time(0))
    print(l.get_edge_matrix_at_time(1))
    print(l.get_edge_matrix_at_time(2))
    print(l.get_edge_matrix_at_time(3))
