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
        #print node
        if problem.isGoalState(node):
            return actions
        if problem.getSuccessors(node) == 0:
			print "OPA!"
			return actions


        for coord, direction, custo in problem.getSuccessors(node):
            custo_vizinho = heuristic(coord, problem)
            i=0
            if custo_atual >= custo_vizinho:
                if i==0:
                    new_actions = actions + [direction]
                    #print new_actions
                    #score = problem.getCostOfActions(new_actions) + heuristic(coord, problem)
                    fila.push( (coord, new_actions), custo_vizinho)
                else:
                    while i!=0:
                        i-=1
                        node, actions = fila.pop()

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

def temperaSimulada(problem, heuristic=nullHeuristic):


    Lista_fechada = []
    fila = util.PriorityQueue()
    filaAux = util.PriorityQueue()
    start = problem.getStartState()
    fila.push( (start, []), heuristic(start, problem))

    temp = 10000
    # Cooling rate
    coolingRate = 0.003
    i=0
    while not fila.isEmpty():
          node, actions = fila.pop()
          custo_atual = heuristic(node, problem)
          #print node
          if problem.isGoalState(node):
              return actions
          if problem.getSuccessors(node) == 0:
  			print "OPA!"
  			return actions

          segur=0
          for coord, direction, custo in problem.getSuccessors(node):
              custo_vizinho = heuristic(coord, problem)
              print "oi 1"
              if custo_atual >= custo_vizinho:
                  #print "oi 1"
                  new_actions = actions + [direction]
                  #print new_actions
                  #score = problem.getCostOfActions(new_actions) + heuristic(coord, problem)
                  fila.push( (coord, new_actions), custo_vizinho)
                  segur+=1
                  #print segur
                  #print temp
              elif temp>=1:
                  #print "oi 2"
                  temp *= 1-coolingRate
                  #print "oi 2"

                  #retira os empilhados
                  if segur>=1:
                      while i>0:
                        #print "oi 3"
                        #print i
                        i-=1
                        node, actions = fila.pop()

                  j=0
                  #print len(problem.getSuccessors(node))-1
                  rand = random.randint(0, len(problem.getSuccessors(node))-1)
                  for coord, direction, custo in problem.getSuccessors(node):
                     custo_vizinho = heuristic(coord, problem)
                     j+=1
                     #adiciona apenas o sorteado
                     if rand==j and acceptanceProbability(heuristic(node, problem),heuristic(coord, problem),temp)>random.random():
                          new_actions = actions + [direction]
                          fila.push( (coord, new_actions), custo_vizinho)

          i+=1
    return actions

def teste(problem, heuristic=nullHeuristic):



    fila = util.PriorityQueue()
    start = problem.getStartState()
    fila.push( (start, []), heuristic(start, problem))
    node, actions = fila.pop()

    for coord, direction, custo in problem.getSuccessors(node):
        print problem.getSuccessors(node)
        if problem.getSuccessors(node) == 0:
			print "OPA!"
			#return actions



def acceptanceProbability(energy,newEnergy,temperature):
        #If the new solution is better, accept it
        if newEnergy < energy:
            return 1.0

        #If the new solution is worse, calculate an acceptance probability
        aux = math.exp((energy - newEnergy) / temperature)
        return aux
        """

def shedule(k = 20, lam = 5, limit = 100):
      return lambda t: if_(t < limit, k * math.exp(-lam * t), 0)

def temperaSimulada(problem, heuristic=nullHeuristic):
     
    import math
    import sys
    #schedule = exp_schedule()
    #shedule = hillClimbing()
    """
    def simulated_annealing(problem, schedule=exp_schedule()):
    "[Fig. 4.5]"
    current = Node(problem.initial)
    for t in xrange(sys.maxint):
        T = schedule(t)
        if T == 0:
            return current
        neighbors = current.expand(problem)
        if not neighbors:
            return current
        next = random.choice(neighbors)
        delta_e = problem.value(next.state) - problem.value(current.state)
        if delta_e > 0 or probability(math.exp(delta_e/T)):
            current = next
    """
    current = problem.getStartState()
    actions = []
    #fila.push( (start, []), heuristic(start, problem))
    for t in xrange(sys.maxsize):
        T = shedule(t)
        if problem.isGoalState(current):
            return actions
        if T == 0:
          return actions
        neighbors = problem.getSuccessors(current)
        if not neighbors:
          return actions 
        next = random.choice(neighbors)
        #print  next
        nodo, direction, custo = next
        #print nodo
        delta_e = heuristic(nodo, problem) - heuristic(current, problem)
        lambda t: if_(t < 200, 20 * math.exp(-5 * t), 0)
        #if t > 0 :
        temperatura = math.exp(delta_e/2)
        #else : 
        #temperatura = 0
        if delta_e > 0 or probability(temperatura) :
          actions = actions + [direction]
          current = nodo

def probability(temperature):
        #If the new solution is better, accept it
        #if newEnergy < energy:
        #   return True

        #If the new solution is worse, calculate an acceptance probability
      if temperature > 0.1 :
        return True



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
