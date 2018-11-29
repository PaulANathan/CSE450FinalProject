
class RearrangableDynamicSteinerTree(object):

    def __init__(self, num_nodes, edge_costs, num_requests, change_requests,
                 terminals):
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
                where i is the index of a terminal node that is not in the tree
                at that time
            terminals: a boolean list of size num_nodes in which terminal[i] ==
                True if i is in the terminal set
         """

        self.num_nodes = num_nodes
        self.cost = edge_costs


        self.edges_at_time_t = [[]]

    def run_eba_algorithm(self):
        # TODO: write eba algorithm
        pass

    def _add_node(self, node):
        """ Add subroutine """
        # TODO: write add subroutine
        pass

    def _remove_node(self, node):
        """ Remove node subroutine """
        # TODO: write remove subroutine
        pass