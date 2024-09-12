from psi_environment.data.car import Car
from psi_environment.data.action import Action
from psi_environment.data.map_state import MapState
from psi_environment.api.environment_api import EnvironmentAPI
from psi_environment.environment import Environment
import numpy as np

from enum import Enum

class Direction(Enum):
    UP = 0
    DOWN = 1
    RIGHT = 2
    LEFT = 3

class PSOCar(Car):

    def __init__(self, road_key: tuple[int, int], road_pos: int, car_id: int):
        super().__init__(road_key, road_pos, car_id)
        self.runned_pso=False
        self.destination=None
        self.previous_action=None
        

    def manhattan(self,point1,point2):

        p11,p12=self.points[int(point1)]
        p21,p22=self.points[int(point2)]
        return abs(p21-p11)+abs(p22-p12)
   
    def path_length(self,path):
        length=0
        if path[0]!=0:
            return np.inf
        for p1,p2 in zip(path,path[1:]):
            length+=self.manhattan(p1,p2)
        return length
    def pso(self):
        N=200
        MAX_ITER=100
        n_points=len(self.points)
        P=np.array([np.hstack(([0],np.random.permutation(np.arange(1,n_points)))) for _ in range(N)])#positions
        
        pbest=np.full((N,n_points),np.inf)
        gbest=np.full((n_points,),np.inf)
    
        for it in range(MAX_ITER):
            print(f"ITERACJA {it}")

            for i,vector in enumerate(P):
                current=self.path_length(vector)
                if np.any(pbest[i]==np.inf) or current<self.path_length(pbest[i]) :
                    pbest[i]=vector 
                if np.any(gbest==np.inf) or current<self.path_length(gbest):
                    gbest=vector 
            for i,vector in enumerate(P):
                
                if np.all(pbest[i]==vector):
                    pdiff=np.array([])
                else:
                    pdiff=np.where(vector!=pbest[i])[0]
                if len(pdiff)>0:    
                    ii,jj=pdiff[:2]
                    vector[ii],vector[jj]=vector[jj],vector[ii]
                gdiff=np.where(vector!=gbest)[0]
                if len(gdiff)>0:
                    ii,jj=gdiff[:2]
                    vector[ii],vector[jj]=vector[jj],vector[ii]
            
                P[i]=vector  
        return gbest,self.path_length(gbest)
        
    def get_action(self, map_state: MapState) -> Action:
        api=EnvironmentAPI(map_state)
        id = self.get_car_id()
        coords=self.get_road_key()

        if not self.runned_pso:
            my_road_pos = self.get_road_pos()
            my_road = api.get_road(coords)
            self.points=[my_road.get_map_position(my_road_pos)]+[point.map_position for point in api.get_points_for_specific_car(id) ]
            self.path,cost=self.pso()
            print(self.path,cost)
            print(np.array(self.points)[self.path])
            self.runned_pso=True
        #TODO: przemieścić pojazd po ścieżce w self.path
        if self.destination==None:
            self.destination=self.path[1]
            print(self.destination)
            self.path=self.path[1:]
        # print(self.api.get_next_road(coords,Action.FORWARD).get_map_position(self.get_road_pos()))
        my_road_pos = self.get_road_pos()
        my_road = api.get_road(coords)
        map_position=my_road.get_map_position(my_road_pos)
        next_point=self.points[self.destination]
        print(map_position,self.destination,next_point)
        print( my_road.get_available_turns(), my_road.is_position_road_end(my_road_pos))
        direction=self.get_current_direction(my_road.get_key() )
        print(direction)
        if map_position==next_point or len(api.get_points_for_specific_car(id))<len(self.path):
            self.destination=self.path[1]
            print(self.destination)
            self.path=self.path[1:]
            next_point=self.points[self.destination]

        if direction==Direction.LEFT and map_position[0]<next_point[0] or direction==Direction.RIGHT and map_position[0]>next_point[0]:
            return Action.BACK

        elif  my_road.is_position_road_end(my_road_pos) and direction==Direction.RIGHT and map_position[1]>next_point[1]:
            return Action.LEFT
        elif my_road.is_position_road_end(my_road_pos) and direction==Direction.UP and map_position[1]>next_point[1] and map_position[0]==next_point[0]:
            return Action.LEFT if Action.LEFT in  my_road.get_available_turns() else Action.RIGHT
        elif my_road.is_position_road_end(my_road_pos) and direction==Direction.LEFT and map_position[1]<next_point[1]:
            return Action.LEFT
        elif direction==Direction.LEFT and map_position[0]>next_point[0]:
            return Action.FORWARD
        elif my_road.is_position_road_end(my_road_pos) and direction==Direction.RIGHT and map_position[1]<next_point[1]:
            return Action.RIGHT
        elif  map_position[1]<next_point[1] and Direction.DOWN:
            return Action.FORWARD
        # elif my_road.is_position_road_end(my_road_pos) and Direction.DOWN  and map_position[0]<next_point[0]:
        #     return Action.LEFT
        elif my_road.is_position_road_end(my_road_pos) and Direction.DOWN  and map_position[0]>next_point[0]:
            return Action.RIGHT
        return Action.FORWARD
    def get_current_direction(self,road_key:tuple[int,int]):
        q,w=road_key
        if (q-w)==6:
            return Direction.UP
        elif (w-q)==6:
            return Direction.DOWN
        elif q>w:
            return Direction.LEFT
        elif q<w:
            return Direction.RIGHT
        
    
if __name__ == "__main__":
    env = Environment(
        agent_type=PSOCar,
        ticks_per_second=10,
        n_bots=1,
        n_points=10,
        traffic_lights_length=10,
        random_seed=2137,
        
    )
    while env.is_running():
        current_cost, is_running = env.step()
        print(current_cost, is_running)