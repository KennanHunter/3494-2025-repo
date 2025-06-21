package frc.robot.subsystems.superstructure;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.shortestpath.BFSShortestPath;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleGraph;

public class SuperStructureMachine {
  private static Graph<KnownState, DefaultEdge> graph = createGraph();

  private static Graph<KnownState, DefaultEdge> createGraph() {
    var graph = new SimpleGraph<KnownState, DefaultEdge>(DefaultEdge.class);

    graph.addEdge(KnownState.Test, KnownState.Test2);
    graph.addEdge(KnownState.Test, KnownState.SafeReset);

    return graph;
  }

  public static GraphPath<KnownState, DefaultEdge> traverse(KnownState start, KnownState end) {
    BFSShortestPath<KnownState, DefaultEdge> path =
        new BFSShortestPath<KnownState, DefaultEdge>(SuperStructureMachine.graph);

    return path.getPath(start, end);
  }
}
