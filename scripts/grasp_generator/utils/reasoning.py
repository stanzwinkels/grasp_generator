#!/usr/bin/env python

# Prolog
import roslib; roslib.load_manifest("rosprolog")
from knowrob_intro.prolog_query import PrologQuery
from rosprolog_client import Prolog

# Ros
import rospy
import ast 
import pdb

class PrologFunctionTask(): 
        def __init__(self): 
                self.pap = "http://www.semanticweb.org/stanz/primitive-ontol#"
                self.load = True
        def feedback(self):
                print("do")
        
        def shape_selection(self, superquadrics, product, task):
                pap = "http://www.semanticweb.org/stanz/primitive-ontol#"
                pq = PrologQuery()


                if self.load:
                        pq.prolog_query("retractall(shape_to_function_to_task:primitive_values(ID,Shape,Scale,Pos)).")      # clear loaded primitives
                        for id, superquadric in enumerate(superquadrics): 
                                shape = superquadric[0:2]
                                scale = superquadric[2:5]
                                pos = superquadric[9:12]
                                # print("assert(quadrics:primitive_values("+ str(id+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos)) +")).")
                                pq.prolog_query("assert(shape_to_function_to_task:primitive_values("+ str(id+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos)) +")).")

                query = pq.prolog_query("shape(ID, Component).")
                print("shape",pq.get_all_solutions(query))

                # query = pq.prolog_query("shape(ID, Component).")
                # print("shape",pq.get_all_solutions(query))

      
                query = pq.prolog_query("select_primitive('"+pap + product+"', '"+pap + task+"', ID, _).")
                print("select_primitive", pq.get_all_solutions(query))
                valid_primitives = pq.get_all_solutions(query)
                return ast.literal_eval(valid_primitives[0])


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
                                scale = superquadric[2:5]
                                pos = superquadric[9:12]
                                pq.prolog_query("assert(shape_to_task:primitive_values("+ str(id+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos)) +")).")

                query = pq.prolog_query("task_prim(ID, Task).")
                print("shape",pq.get_all_solutions(query))


                print("finished loading...")

                return

# class PrologTask(): 
#         def __init__(self):
#                 self.pap = XX
#         def feedback():
        
#         def shape_selection():
        
#         def task_validation():
        
        









def prolog_version1_rules(superquadrics, object, task):
    # pap = "http://www.semanticweb.org/stanz/thesis-ontol#"
    pap = "http://www.semanticweb.org/stanz/primitive-ontol#"
    pq = PrologQuery()

    if load is True:
        pq.prolog_query("retractall(method_affordance:primitive_values(X,Y,Z,W)).")      # clear loaded primitives
        for id, superquadric in enumerate(superquadrics): 
            shape = superquadric[0:2]
            scale = superquadric[2:5]
            pos = superquadric[9:12]
            # print("assert(quadrics:primitive_values("+ str(id+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos)) +")).")
            pq.prolog_query("assert(method_affordance:primitive_values("+ str(id+1) +", "+str(list(shape))+", "+ str(list(scale)) +", "+ str(list(pos)) +")).")


    query = pq.prolog_query("shape(ID, Component).")
    print("shape",pq.get_all_solutions(query))

    product = "Limonade"
    task = "Pour"

    query = pq.prolog_query("select_primitive('"+pap + product+"', '"+pap + task+"', ID, _).")
    print("select_primitive", pq.get_all_solutions(query))
    valid_primitives = pq.get_all_solutions(query)

    return valid_primitives


if __name__ == "__main__":
    rospy.init_node("example_task")
    start = rospy.get_rostime()


    prolog_quadric()
    print("-------- Finished -------- ")
