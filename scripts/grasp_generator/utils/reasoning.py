#!/usr/bin/env python

# Prolog
import roslib; roslib.load_manifest("rosprolog")
from knowrob_intro.prolog_query import PrologQuery
from rosprolog_client import Prolog

# Ros
import rospy
import ast 
import pdb
import unicodedata
import warnings
import re

class PrologFunctionTask(): 
        def __init__(self): 
                self.pap = "http://www.semanticweb.org/stanz/primitive-ontol#"
                self.load = True
        def feedback(self):
                print("do")
        
        def shape_selection(self, superquadrics, product, task, semantic_shapes):
                task = 'Wgrasp'
                pap = "http://www.semanticweb.org/stanz/primitive-ontol#"
                pq = PrologQuery()

                if self.load:
                        pq.prolog_query("retractall(shape_to_function_to_task:primitive_values(ID,Shape,Scale,Pos,Semantic_shape)).")      # clear loaded primitives
                        for id, superquadric in enumerate(superquadrics): 
                                shape = superquadric[0:2]
                                scale = superquadric[2:5]*2
                                pos = superquadric[9:12]
                                pq.prolog_query("assert(shape_to_function_to_task:primitive_values("+ str(id+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos))+", "+ str(semantic_shapes[id]) +")).")

                query = pq.prolog_query("affordance(ID, Region, Affordance).")
                results = pq.get_all_solutions(query)

                # query = pq.prolog_query("primitive_values(ID, Eps, _, _,_), shape_identification(Eps, Shape).")
                # forms = pq.get_all_solutions(query)
                try: 
                        if results[0] == 'false':
                                raise ValueError("No feasible task found", results[0])
                        for idx, result in enumerate(results[1::3]):
                                wordlist = []
                                words = result.split()
                                wordlist.extend(words)
                                for word in wordlist:
                                        try:
                                                found_task = str(word)
                                        except:
                                                found_task = re.findall(r"'(.*?)'", word, re.DOTALL)[0]
                                        if found_task == task:
                                                # print("TASK FOUND")
                                                Id = int(results[2::3][idx])
                                                region = results[::3][idx]
                                                break
                except:
                        Id = 1
                        region = 'All'
                return Id, region
      
                # query = pq.prolog_query("select_primitive('"+pap + product+"', '"+pap + task+"', ID, _).")
                # print("select_primitive", pq.get_all_solutions(query))
                # valid_primitives = pq.get_all_solutions(query)
                # return ast.literal_eval(valid_primitives[0])


class PrologShapeTask():
        def __init__(self): 
                self.pap = "http://www.semanticweb.org/stanz/primitive-ontol#"
                self.load = True        

        def feedback(self):
                print("do")
        
        def shape_selection(self, superquadrics, product, task):
                pap = "http://www.semanticweb.org/stanz/primitive-ontol#"
                pq = PrologQuery()
        
                if self.load:
                        pq.prolog_query("retractall(shape_to_task:primitive_values(ID,Shape,Scale,Pos)).")      # clear loaded primitives
                        for id, superquadric in enumerate(superquadrics): 
                                shape = superquadric[0:2]
                                scale = superquadric[2:5]*2
                                pos = superquadric[9:12]
                                pq.prolog_query("assert(shape_to_task:primitive_values("+ str(id+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos)) +")).")

                print("\nFound tasks: ")

                query = pq.prolog_query("task_prim(ID, Task).")
                results = pq.get_all_solutions(query)


                query = pq.prolog_query("primitive_values(ID, Eps, _, _), shape_identification(Eps, Shape).")
                forms = pq.get_all_solutions(query)

                try: 
                        for idx, result in enumerate(results):
                                result = unicodedata.normalize('NFKD', result).encode('ascii', 'ignore')
                                if result == task:
                                        ID = int(results[idx+1])
                                        break
                except:
                        warnings.warn(" #### NO SUITABLE TASK HAS BEEN FOUND! ####")
                        ID = 1
                                                              
                return ID, forms[3-3::3]


        




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


if __name__ == "__main__":
    rospy.init_node("prolog_rules")
    start = rospy.get_rostime()

#     prolog_quadric()
    print("-------- Finished -------- ")
