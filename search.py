# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util
import random
import math
from game import Directions

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).

  You do not need to change anything in this class, ever.
  """

  def getStartState(self):
     """
     Returns the start state for the search problem
     """
     util.raiseNotDefined()

  def isGoalState(self, state):
     """
       state: Search state

     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state

     For a given state, this should return a list of triples,
     (successor, action, stepCost), where 'successor' is a
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take

     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()


def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  n = Directions.NORTH
  e = Directions.EAST
  return  [s,s,w,s,w,w,s,n,n,w,n,n,e]

def depthFirstSearch(problem):
  "*** YOUR CODE HERE ***"
  #util.raiseNotDefined()
  pilha = util.Stack()
  pilha.push( (problem.getStartState(), [], []) )
  while not pilha.isEmpty():
        node, actions, visited = pilha.pop()

        for coord, direction, custo in problem.getSuccessors(node):
            if not coord in visited:
                if problem.isGoalState(coord):
                    return actions + [direction]
                pilha.push((coord, actions + [direction], visited + [node] ))

  return []



def depthaaFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 85].

  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.7].

  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:

  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  #print "Start:", problem.getStartState()
  #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  #print "Start's successors:", problem.getSuccessors(problem.getStartState())

  node = problem.getStartState();
  cutoff = problem.isGoalState(node);
  sucessors = problem.getSuccessors(node);
  cutoff_occurred = False

  lista = sucessors.pop()
  dx, dy = lista[0]
  #dy = lista.pop()
  print "Start:", node
  print "Is the start a goal?", cutoff
  print "Start's successors:", lista[1]
  node = lista[0]
  cutoff = problem.isGoalState(node);
  sucessors = problem.getSuccessors(node);
  print "Start:", node
  print "Is the start a goal?", cutoff
  print "Start's successors:", sucessors
  depth_limited_search(problem, limit=50)
  action = []
  action.append(node)
  action.append(lista[0])
  teste = (dx, dy)
  #return action
  from game import Directions

  s = Directions.SOUTH
  w = Directions.WEST

  #return [s,s,w,s,w,w,s,w]
  return [Directions.SOUTH,Directions.SOUTH,Directions.WEST,Directions.SOUTH,Directions.WEST,Directions.WEST,Directions.SOUTH,Directions.WEST]

# util.raiseNotDefined()
def depth_limited_search(problem, limit=50):
    "[Fig. 3.17]"
    cont = 0
    def recursive_dls(node, problem, limit, cont):
        if problem.isGoalState(node):
            cont += 1
            return node
        elif len(Stack.list) == limit:
			return 'cutoff'
        else:
            cutoff_occurred = False
            print "Start:", node
            sucessors = problem.getSuccessors(node)
            #lista = sucessors.pop()
            for child in sucessors.pop():
				Stack.push(child)
				result = recursive_dls(child, problem, limit, cont)
				if result == 'cutoff':
				   cutoff_occurred = True
				elif result is not None:
					return result
            return if_(cutoff_occurred, 'cutoff', None)

    # Body of depth_limited_search:
    return recursive_dls(problem.getStartState(), problem, limit, cont)

  #node = problem.getStartState();
  #cutoff = problem.isGoalState(node);
  #sucessors = problem.getSuccessors(node);

def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 81]"
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  #util.raiseNotDefined()
  Lista_fechada = []
  fila = util.PriorityQueue()
  start = problem.getStartState()
  fila.push( (start, []), heuristic(start, problem))

  while not fila.isEmpty():
        node, actions = fila.pop()

        if problem.isGoalState(node):
            return actions

        Lista_fechada.append(node)

        for coord, direction, custo in problem.getSuccessors(node):
            if not coord in Lista_fechada:
                new_actions = actions + [direction]
                score = problem.getCostOfActions(new_actions) + heuristic(coord, problem)
                fila.push( (coord, new_actions), score)

  return []

def hillClimbing(problem, heuristic=nullHeuristic):
  #util.raiseNotDefined()
  Lista_fechada = []
  fila = util.PriorityQueue()
  start = problem.getStartState()
  fila.push( (start, []), heuristic(start, problem))

  while not fila.isEmpty():
        node, actions = fila.pop()
        custo_atual = heuristic(node, problem)

        if problem.isGoalState(node):
            return actions
        if problem.getSuccessors(node) == 0:
			print "OPA!"
			return actions


        for coord, direction, custo in problem.getSuccessors(node):
            custo_vizinho = heuristic(coord, problem)
            if custo_atual >= custo_vizinho:
                new_actions = actions + [direction]
                print new_actions
                #score = problem.getCostOfActions(new_actions) + heuristic(coord, problem)
                fila.push( (coord, new_actions), custo_vizinho)

  return actions

  """
	Hill-Climbing(Problema) retorna um estado que e o maximo local
	EstadoAtual <- FazNao(Problema[EstadoInicial])
	loop do
	Vizinho <- SucessorDeMaiorValor(EstadoAtual)
	se Vizinho[Valor] for menor ou igual EstadoAtual[Valor] entao
		retorna EstadoAtual
	EstadoAtual <- Vizinho

	FimVizinho[2] < Atual[2]
	f hill_climbing(problem):

    From the initial node, keep choosing the neighbor with highest value,
    stopping when no neighbor is better. [Fig. 4.2]
    current = Node(problem.initial)
    while True:
        neighbors = current.expand(problem)
        if not neighbors:
            break
        neighbor = argmax_random_tie(neighbors,
                                     lambda node: problem.value(node.state))
        if problem.value(neighbor.state) <= problem.value(current.state):
            break
        current = neighbor
    return current.state

	current = problem.getStartState()
	while True:
		neighbors = current.getSuccessors(problem)
		if not neighbors:
			break

		neighbor =

	Lista_fechada = []
	fila = util.PriorityQueue()
	start = problem.getStartState()
	#Atual = (start, []), heuristic(start, problem)
	fila.push( (start, []), heuristic(start, problem) )
	Atual = fila.pop()
	fila.push(self, Atual)
	while not fila.isEmpty():
		Vizinho = fila.pop()

		if problem.isGoalState(Atual[0]):
			return node
		elif Vizinho[2] < Atual[2]:
			return actions
        else:
			Atual = Vizinho

			Lista_fechada.append(node)

        for coord, direction, custo in problem.getSuccessors(node):
            if not coord in Lista_fechada:
                new_actions = actions + [direction]
                score = problem.getCostOfActions(new_actions) + heuristic(coord, problem)
                Vizinho = ((coord, new_actions), score)
                fila.push( Vizinho )

	return []
	"""







def temperaSimulada(problem, heuristic=nullHeuristic):



    s = Directions.SOUTH
    w = Directions.WEST
    n = Directions.NORTH
    e = Directions.EAST

    Lista_fechada = []
    fila = util.PriorityQueue()
    start = problem.getStartState()
    fila.push( (start, []), heuristic(start, problem))

    temp = 10000
    # Cooling rate
    coolingRate = 0.003

    while temp>1:

        scoreOld=0
        score=0
        solucaoAtual=[]
        solucao=[]
        melhorSolucao=[]
        while not fila.isEmpty():
              print "oi1"
              node, actions = fila.pop()
              custo_atual = heuristic(node, problem)
              solucao=actions
              scoreOld=custo_atual
              if problem.isGoalState(node):
                  print "oi"
                  return melhorSolucao
              if problem.getSuccessors(node) == 0:
                  print "OPA!"
                  return melhorSolucao

              i=0
              vetor = []
              #print problem.getSuccessors(node)

              for coord, direction, custo in problem.getSuccessors(node):
                  custo_vizinho = heuristic(coord, problem)
                  pair=coord,direction
                  vetor.append(pair)
                  i+=1
                  #print "oi 2"
                  if i==len(problem.getSuccessors(node)):
                      #print "oi 3"
                      valueInfo,direct = vetor[random.randint(0, len(vetor)-1)]

                      new_actions = actions + [direct]
                      solucaoAtual = new_actions
                      score = heuristic(valueInfo, problem)
                      #print score
                      fila.push( (coord, new_actions), custo_vizinho)


        if acceptanceProbability(score, scoreOld, temp)>random.random():
          acoes=solucaoAtual

        if score<scoreOld:
          melhorSolucao = solucaoAtual
        print temp
        temp *= 1-coolingRate
    print melhorSolucao
    return melhorSolucao

def acceptanceProbability(energy,newEnergy,temperature):
        #If the new solution is better, accept it
        if newEnergy < energy:
            return 1.0

        #If the new solution is worse, calculate an acceptance probability
        aux = math.exp((energy - newEnergy) / temperature)
        return aux


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
