
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


    def run_eba_algorithm(self):
        """ Runs the Edge Bounded algorithm outlined in the paper Dynamic
         Steiner Tree Problem. Imase & Waxman (1989) """
        # TODO: write eba algorithm
        i = 0

        for i in range(1, self.num_requests+1):
            # Si-1
            vertice_set = self.tree_nodes_t[i-1].copy()
            terminal_set = self.terminals_t[i-1].copy()
            edge_matrix = self.edge_matrix_t[i-1].copy()

            ri, command = self.change_requests[i]
            if command == "add":
                # Si = Si-1 U {ri}
                if ri not in vertices:
                    vertices.append(ri)

                # add(ri, T, S)
                self._add_node(ri, vertice_set, terminal_set, edge_matrix)

            elif command == "remove":

                # Si = Si-1 - {ri}
                if ri in vertices:
                    index = index_of(i, vertices)
                    vertices = vertices[0, index] + vertices[index+1:]


                self._remove_node(ri, vertice_set, terminal_set, edge_matrix)
            else:
                raise ValueError("unrecognized command {0}".format(command))

        pass

    def _add_node(self, node, vertice_set, terminal_set, edge_matrix):
        """ Add subroutine """
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
        _, w1 = edges[0]
        # add (v, w1) to edge_matrix
        edge_matrix[node, w1] = 1
        edge_matrix[w1, node] = 1
        # add v to vertice_set
        vertice_set += [w1]
        # subtract (v, w1) from W
        edges = edges[1:]
        # while W is not empty
        while len(edges) > 0:
            # find the minimum edge in W (v, w1)
            _, w1 = edges[0]
            # remove (v, w1) from W
            edges = edges[1:]
            # find the single path connecting (v, w1) i.e p(v, w1; T);
            path = self.get_path_connecting(node, w1, edge_matrix)
            # find the maximum edge in that path e
            max_edge, max_cost = self.max_edge_in_path(path, edge_matrix)

            # if cost(e) > delta * cost(v, w1) ->
            if cost_max > delta * self.cost[node][w1]:
                # T := T - e + (v, w1)
                # remove e from edge_matrix
                edge_matrix[max_edge[0]][max_edge[1]] = 0
                edge_matrix[max_edge[1]][max_edge[0]] = 0
                # add (v, w1) from edge_matrix
                edge_matrix[node][w1] = 1
                edge_matrix[w1][node] = 1

        self._remove_node(vertice_set, terminal_set, edge_matrix)

    def _remove_node(self, vertice_set, terminal_set, edge_matrix):
        """ Remove node subroutine """
        # TODO: write remove subroutine
        # let W be the set of vertices_set - terminal_set
        non_terminals = [node for node in vertice_set if node not in terminal_set]
        # for each node in W
        for node in non_terminals:
            if self.degree(node, edge_matrix) == 1:
                # remove node
                pass
            if self.degree(node, edge_matrix) == 2:
                # x1 and x2 are nodes adjacent to node
                [x1, x2] = [i for i in range(len(edge_matrix[node])) if edge_matrix[node][i] == 1]
                # for each u1_node in x1 connected component
                for u1_node in self.connected_component(x1, edge_matrix):
                    # for each u2_node in x2 connected component
                    for u2_node in self.connect_component(x2, edge_matrix):
                        # find the min g(u1_node, u2_node)
                        # g = max edge cost on path from u1_node to u2_node
                        path = self.get_path_connecting(u1_node, u2_node, edge_matrix)
                        edge, cost = max_edge_in_path(path, edge_matrix)


    def get_path_connecting(self, u, v, edge_matrix):
        pass

    def connected_component(self, node, edge_matrix):
        pass

    def max_edge_in_path(self, path):
        max_edge = path[0]
        max_cost = self.cost[max_edge[0]][max_edge[1]]
        for i in range(1, len(path)):
            cost = self.cost[path[i][0]][path[i][1]]
            if cost > max_cost:
                max_cost = cost
                max_edge = path[i]
        return max_edge, max_cost

    def degree(self, node, edge_matrix):
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