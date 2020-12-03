using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System.Xml;

// Amalgamation of classes that parse an XML file into 
// Unity game objects and a walkable graph.  An A* pathing
// algorithm is then applied to the graph and colors the in
// scene node game objects.
public class SceneLoader : MonoBehaviour
{
	[SerializeField] TextAsset graphXML;

	// Initializes walkable graph from XML text.
	// Constructs the objects in the scene.
	// Applies A* pathing algorithm to path.
	void Start()
	{
		var graph = new Graph(graphXML.text);
		DrawScene(graph);
		foreach (Path path in graph.paths)
			A_starPathing(path.start, path.end, graph);
	}

	// Data type that holds coordinates and connections
	// to other nodes.
	public class Node
	{
		public string name;
		public float x, y, z;
		public float cost = Mathf.Infinity;
		public Node from;
		public List<Edge> edgesOut = new List<Edge>();
	}

	// Data type that points to two nodes and holds an
	// associated cost.
	public class Edge
	{
		public Node start;
		public Node end;
		public float cost;
	}

	// Data type that specifies two nodes that will be
	// connected by a pathing algorithm.
	public class Path
	{
		public Node start;
		public Node end;
	}

	// A walkable graph populated with a list of nodes,
	// a list of edges, and a list of paths read from 
	// an XML document.
	public class Graph
	{
		public List<Node> nodes = new List<Node>();
		public List<Edge> edges = new List<Edge>();
		public List<Path> paths = new List<Path>();

		// Accepts an XML string to parse in and convert to usable
		// data structures.
		public Graph(string xml)
		{
			Node startNode;
			Node endNode;
			XmlDocument xmlDoc = new XmlDocument();
			xmlDoc.LoadXml(xml);

			// Parse, create, and add nodes to graph list.
			foreach (XmlElement elem in xmlDoc.SelectNodes("Graph/Node"))
			{
				nodes.Add(new Node
				{
					name = elem.GetAttribute("Name"),
					x = float.Parse(elem.GetAttribute("X")),
					y = float.Parse(elem.GetAttribute("Y")),
					z = float.Parse(elem.GetAttribute("Z"))
				});
			}

			// Parse, create, and add edges to graph list and add outward edges on nodes.
			foreach (XmlElement elem in xmlDoc.SelectNodes("Graph/Edge"))
			{
				startNode = nodes.Where(n => n.name.Equals(elem.GetAttribute("Start"))).FirstOrDefault();
				endNode = nodes.Where(n => n.name.Equals(elem.GetAttribute("End"))).FirstOrDefault();
				var newEdge = new Edge
				{
					start = startNode,
					end = endNode,
					cost = float.Parse(elem.GetAttribute("Cost"))
				};
				edges.Add(newEdge);
				if (!startNode.edgesOut.Contains(newEdge))
					startNode.edgesOut.Add(newEdge);
				if (!endNode.edgesOut.Contains(newEdge))
					endNode.edgesOut.Add(newEdge);
			}

			// Parse, and add paths to graph list.
			foreach (XmlElement elem in xmlDoc.SelectNodes("Graph/Path"))
			{
				startNode = nodes.Where(n => n.name.Equals(elem.GetAttribute("Start"))).FirstOrDefault();
				endNode = nodes.Where(n => n.name.Equals(elem.GetAttribute("End"))).FirstOrDefault();
				paths.Add(new Path
				{
					start = startNode,
					end = endNode
				});
			}
		}
	}

	// Uses the walkable graph to create game objects within the scene
	void DrawScene(Graph graph)
	{
		// Creates primitives at specified coordinates, gives the object a name and color
		foreach (Node node in graph.nodes)
		{
			GameObject newNode = GameObject.CreatePrimitive(PrimitiveType.Sphere);

			newNode.transform.position = new Vector3(node.x, node.y, node.z);
			newNode.name = node.name;
			newNode.GetComponent<Renderer>().material.color = Color.red;
		}

		// Creates primitives between corresponding nodes, adjusts scale and angle,
		// and colors the objects.
		foreach (Edge edge in graph.edges)
		{
			GameObject newEdge = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
			GameObject nodeStart = GameObject.Find(edge.start.name);
			GameObject nodeEnd = GameObject.Find(edge.end.name);

			newEdge.name = nodeStart.name + nodeEnd.name;
			newEdge.transform.position = (nodeEnd.transform.position + nodeStart.transform.position) / 2f;
			newEdge.transform.LookAt(nodeEnd.transform.position);
			newEdge.transform.Rotate(new Vector3(90f, 0f, 0f));
			newEdge.transform.localScale = new Vector3(0.25f, Vector3.Magnitude(nodeEnd.transform.position
				- nodeStart.transform.position), 0.25f) / 2f;
			newEdge.GetComponent<Renderer>().material.color = Color.red;
		}
	}

	// Function accepts a starting node, ending node, and a walkable graph to
	// apply A* pathing algorithm to.  Nodes are colored blue along the decided
	// best path.
	void A_starPathing(Node startNode, Node endNode, Graph graph)
	{
		List<Node> PriorityQueue = new List<Node>();
		List<Node> VisitedNodes = new List<Node>();
		List<Node> NodePath = new List<Node>();
		Node current = startNode;
		float dist;

		// Initialize first node in path in the queue
		PriorityQueue.Add(startNode);
		current.cost = 0;

		// Iterate through the nodes searching for shortest path
		while (PriorityQueue.Count != 0)
		{
			current = PriorityQueue[0];
			foreach(Edge edge in current.edgesOut)
			{
				// Reference each node connected by outward edges
				Node neighbour = GetNeighbourNode(current, edge);
				if (VisitedNodes.Contains(neighbour))
					continue;

				// Finds the total current path cost plus distance to end
				dist = current.cost + edge.cost + GetDistanceToEnd(current, endNode);
				if (dist >= neighbour.cost) // default cost of neighbour is infinite
					continue;

				// New neighbour found, assign new values associated with best path
				else
				{
					neighbour.cost = dist;
					neighbour.from = current;
					PriorityQueue.Add(neighbour);
				}
			}
			VisitedNodes.Add(current);
			PriorityQueue.Remove(current);
		}
		// Begin working backwards from end node to determine best path
		Node backNode = endNode;

		while (backNode != startNode)
		{
			NodePath.Add(backNode);
			backNode = backNode.from;
		}
		// Manually add startNode to list
		NodePath.Add(startNode);

		// Iterate through best path and change color of game objects
		foreach (Node elem in NodePath)
		{
			GameObject node = GameObject.Find(elem.name);
			node.GetComponent<Renderer>().material.color = Color.blue;
		}
	}

	// Given a current node and edge will return the node on the other end
	// of the edge
	Node GetNeighbourNode(Node current, Edge edge)
	{
		if (current.name == edge.start.name)
			return edge.end;
		return edge.start;
	}

	// Return the distance from a node to the end node
	float GetDistanceToEnd(Node current, Node end)
	{
		Vector3 a = new Vector3(current.x, current.y, current.z);
		Vector3 b = new Vector3(end.x, end.y, end.z);
		float dist = Vector3.Distance(a, b);
		return dist;
	}
}
