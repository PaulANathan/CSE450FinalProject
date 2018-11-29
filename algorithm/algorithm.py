
class RearrangableDynamicSteinerTree(object):

    def __init__(self, num_nodes, edge_costs, num_requests, change_requests,
                 terminals, initial_node):
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
         """

        self.num_nodes = num_nodes
        self.cost = edge_costs

        self.is_terminal = terminals

        # initially there is a single node in the tree
        self.tree_nodes_t = [[initial_node]]
        # initially there are no edges in the tree
        self.edges_at_time_t = [[]]

        # a list of adjacency matrix for each time i
        # after run_eba_algorithm is called, this will contain num_requests + 1
        # adjacency matrices, one for each step of the algorithm
        self.edge_active = [ [[0 for i in range(self.num_nodes)] for j in
                             range(self.num_nodes)] ]

        self.num_requests = num_requests
        self.change_requests = change_requests

    def run_eba_algorithm(self):
        """ Runs the Edge Bounded algorithm outlined in the paper Dynamic
         Steiner Tree Problem. Imase & Waxman (1989) """
        # TODO: write eba algorithm
        i = 0

        for i in range(1, self.num_requests+1):
            tree = self.tree_nodes_t[i-1]

            ri, command = self.change_requests[i]
            if command == "add":
                self._add_node(ri, tree, term_set=None)
            elif command == "remove":
                self._remove_node(ri, tree, )
            else:
                raise ValueError("unrecognized command {0}".format(command))

        pass

    def _add_node(self, node, tree, term_set):
        """ Add subroutine """
        # TODO: write add subroutine
        pass

    def _remove_node(self, node):
        """ Remove node subroutine """
        # TODO: write remove subroutine
        pass