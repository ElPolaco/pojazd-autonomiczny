from psi_environment.data.car import Car
from psi_environment.data.action import Action
from psi_environment.data.map_state import MapState
from psi_environment.api.environment_api import EnvironmentAPI
from psi_environment.environment import Environment
import numpy as np

class PSOCar(Car):
    def __init__(self, road_key: tuple[int, int], road_pos: int, car_id: int):
        super().__init__(road_key, road_pos, car_id)
        self.runned_pso=False

    def manhattan(self,point1,point2):
        # print(f"XDD{(self.points[point1].map_position)}")
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
                # print(vector,current)
                if np.any(pbest[i]==np.inf) or current<self.path_length(pbest[i]) :
                    pbest[i]=vector 
                if np.any(gbest==np.inf) or current<self.path_length(gbest):
                    gbest=vector 
            for i,vector in enumerate(P):
                # r1=np.random.random()
                # r2=np.random.random()
                # velocity=X*(V[i]+c1*r1*(pbest[i]-vector)+c2*r2*(gbest-vector))
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
        self.api=EnvironmentAPI(map_state)
        id = self.get_car_id()
        
        if not self.runned_pso:
            self.points=[self.get_road_key()]+[point.map_position for point in self.api.get_points_for_specific_car(id) ]
            self.path,cost=self.pso()
            print(self.path,cost)
            self.runned_pso=True
        coords=self.get_road_key()
        
        return Action.FORWARD if Action.FORWARD in self.api.get_available_turns(coords) else Action.BACK
    
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