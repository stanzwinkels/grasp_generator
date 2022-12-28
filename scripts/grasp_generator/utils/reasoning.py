#!/usr/bin/env python

# Prolog
import roslib; roslib.load_manifest("rosprolog")
from knowrob_intro.prolog_query import PrologQuery
from rosprolog_client import Prolog

# Ros
import rospy
import pdb
import unicodedata
import warnings
import numpy as np 

class PrologFunctionTask(): 
        def __init__(self): 
                # self.pap = "http://www.semanticweb.org/stanz/primitive-ontol#"
                self.pap = "http://www.semanticweb.org/stanz/ontologies/2022/11/affordance-ont#"
                self.load = True
        def feedback(self):
                print("do")
        
        def shape_selection(self, superquadrics, product, task, semantic_shapes):
                task = 'Hammer'
                pap = "http://www.semanticweb.org/stanz/ontologies/2022/11/affordance-ont#"

                pq = PrologQuery()

                if self.load:
                        pq.prolog_query("retractall(shape_to_function_to_task:primitive_values(ID,Shape,Scale,Pos,Semantic_shape)).")      # clear loaded primitives
                        for id, superquadric in enumerate(superquadrics): 
                                shape = superquadric[0:2]
                                scale = superquadric[2:5]*2
                                pos = superquadric[9:12]
                                pq.prolog_query("assert(shape_to_function_to_task:primitive_values("+ str(id+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos))+", "+ str(semantic_shapes[id]) +")).")
                
                query = pq.prolog_query("affordAffordance(ID, Region, Affordance).")
                results = pq.get_all_solutions(query)
                list_id = []    
                list_region = []

                remove_result = np.array([])
                if task == 'HandOver': 
                        query = pq.prolog_query("not_affordAffordance(Affordance, pap:'"+str(task) +"').") 
                        not_affordance = pq.get_all_solutions(query)
                        results = np.reshape(results, (-1,3))
                        for not_afford in not_affordance:                             
                                remove_result = np.append(remove_result, results[results[:,1] == not_afford])
                        remove_result = np.reshape(remove_result, (-1,3))
                        list_id = range(len(superquadrics)) 
                        remove_ids = map(int, remove_result[:,2])
                        remove_ids = (np.array(remove_ids)-1).tolist()
                        number_superquadrics = range(len(superquadrics))
                        list_id = [i for i in number_superquadrics if i not in remove_ids]
                        for id in list_id: 
                                list_region.append('All')

                if not task == 'HandOver':
                        query = pq.prolog_query("task_object(Affordance, pap:'"+str(task) +"').")
                        required_affordance = pq.get_all_solutions(query)
                        if (str(required_affordance[0]) in results and str(required_affordance[1]) in results):
                                query = pq.prolog_query("grasp_affordance(ID, Surface, pap:'"+str(task)+"').")
                                results = pq.get_all_solutions(query)
                                for idx, result in enumerate(results[0::3]):
                                        list_region.append(results[1::2][idx])
                                        list_id.append(int(results[0::2][idx]))
 
 
                return list_id, list_region
                                 

class PrologShapeTask():
        def __init__(self): 
                self.pap = "http://www.semanticweb.org/stanz/primitive-ontol#"
                self.load = True        

        def feedback(self):
                print("do")
        
        def shape_selection(self, superquadrics, product, task, semantic_shapes):
                task = 'HandOver'
                pap = "http://www.semanticweb.org/stanz/ontologies/2022/11/affordance-ont#"

                pq = PrologQuery()
        
                if self.load:
                        pq.prolog_query("retractall(shape_to_task:primitive_values(ID,Shape,Scale,Pos,Semantic_shape)).")      # clear loaded primitives
                        for id, superquadric in enumerate(superquadrics): 
                                shape = superquadric[0:2]
                                scale = superquadric[2:5]*2
                                pos = superquadric[9:12]
                                pq.prolog_query("assert(shape_to_task:primitive_values("+ str(id+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos))+", "+ str(semantic_shapes[id]) +")).")
                query = pq.prolog_query("affordTask(ID, Region, pap:'"+str(task)+"').")
                results = pq.get_all_solutions(query)
                list_id = []    
                list_region = []

                list_region.append(results[0])
                list_id.append(int(results[1]))
                return list_id, list_region


if __name__ == "__main__":
    rospy.init_node("prolog_rules")
    start = rospy.get_rostime()
    print("-------- Finished -------- ")



# check if found affordances match required affordances.. 

# affordance_time = time.time()
# query = pq.prolog_query("task_object(Afford," +str(task) +").")
# affordance = pq.get_all_solutions(query)
# affordance_time = time.time() - affordance_time

# pdb.set_trace()

# new_affordance_list = []
# for afford in affordance: 
#         afford = str(afford)
#         new_affordance_list.append(afford)
# new_affordance_list = np.array(new_affordance_list).reshape(-1,2)
# new_affordance_list[:,[1,0]]= new_affordance_list[:,[0,1]]

# file_codes = {}
# for name, code in new_affordance_list:
#         if name not in file_codes:
#                 file_codes[name] = []
#         file_codes[name].append(code)

# detected_affordance = results[1::3]

# feasible_tasks = []
# for key, value in file_codes.items():
#         some_result = all(elem in detected_affordance for elem in value)
#         if some_result:
#                 feasible_tasks.append(key) 

# list_id = []
# list_region = []
# if task in feasible_tasks:
#         query = pq.prolog_query("grasp_affordance(ID, Surface, "+task+").")
#         results = pq.get_all_solutions(query)
#         print(results)
#         for idx, result in enumerate(results[0::3]):
#                 if task == result:
#                         list_region.append(results[2::3][idx])
#                         list_id.append(int(results[1::3][idx]))



# def prolog_version1_rules(superquadrics, object, task):
#     # pap = "http://www.semanticweb.org/stanz/thesis-ontol#"
#     pap = "http://www.semanticweb.org/stanz/primitive-ontol#"
#     pq = PrologQuery()

#     if load is True:
#         pq.prolog_query("retractall(method_affordance:primitive_values(X,Y,Z,W)).")      # clear loaded primitives
#         for id, superquadric in enumerate(superquadrics): 
#             shape = superquadric[0:2]
#             scale = superquadric[2:5]
#             pos = superquadric[9:12]
#             # print("assert(quadrics:primitive_values("+ str(id+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos)) +")).")
#             pq.prolog_query("assert(method_affordance:primitive_values("+ str(id+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos)) +")).")


#     query = pq.prolog_query("shape(ID, Component).")
#     print("shape",pq.get_all_solutions(query))

#     product = "Limonade"
#     task = "Pour"

#     query = pq.prolog_query("select_primitive('"+pap + product+"', '"+pap + task+"', ID, _).")
#     print("select_primitive", pq.get_all_solutions(query))
#     valid_primitives = pq.get_all_solutions(query)

#     return valid_primitives


# query = pq.prolog_query("task_prim(ID, Task).")
# results = pq.get_all_solutions(query)


# query = pq.prolog_query("primitive_values(ID, Eps, _, _), shape_identification(Eps, Shape).")
# forms = pq.get_all_solutions(query)

# try: 
#         for idx, result in enumerate(results):
#                 result = unicodedata.normalize('NFKD', result).encode('ascii', 'ignore')
#                 if result == task:
#                         ID = int(results[idx+1])
#                         break
# except:
#         warnings.warn(" #### NO SUITABLE TASK HAS BEEN FOUND! ####")
#         ID = 1
                        
