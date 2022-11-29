% example of Prolog module defined by the user
% STATUS: working

% module definition and public (externally visible) predicates
:- module(tasks,
    [
        % perishable/1,
        % nonperishable/1,
        % weight/2,
        % location/2,
        % grasp_location/2,
        % mass/2,
        % result/3,
        width/2,
        depth/2,
        height/2,
        obj_shape/2,
        id_shape/2,
        object/2,
        region/2,
        task/2,
        goal/3,
        aruc_offset/4,
        dimension/4
    ]).




width(O, D) :- triple(O, pap:'hasWidth', D).
depth(O, D) :- triple(O, pap:'hasLength', D).
height(O, D) :- triple(O, pap:'hasHeight', D).

% Get the dimension of the object.
dimension(Id, W, L, H) :-    triple(T, pap:'hasArucoID', Id),
                            triple(T, pap:'hasWidth', W),
                            triple(T, pap:'hasLength', L),
                            triple(T, pap:'hasHeight', H).

% Get the dimension of the object.
aruc_offset(Id, X, Y, Z) :-   triple(T, pap:'hasArucoID', Id),
                            triple(T, pap:'hasOffset_X', X),
                            triple(T, pap:'hasOffset_Y', Y),
                            triple(T, pap:'hasOffset_Z', Z).


% Check object type belonging to an ID
obj_shape(O, S) :- triple(O, _, D), 
                    triple(D, _, pap:'isShapeOf'),
                    triple(D, _, S), 
                    subclass_of(S,pap:'ShapePrimitive').


% Shape of the ID
id_shape(Id, S) :- triple(T, pap:'hasArucoID', Id),
                instance_of(T,O),
                triple(O, _, D), 
                triple(D, _, pap:'isShapeOf'),
                triple(D, _, S), 
                subclass_of(S,pap:'ShapePrimitive').



% Shape of the ID
object(O, Id) :- triple(T, pap:'hasArucoID', Id),
                instance_of(T,O).


% % Check shape of the object.
% shape(Id,S) :- triple(T, pap:'hasArucoID', Id),
%                 instance_of(T,O),
%                 subclass_of(O,S), 
%                 subclass_of(S,pap:'ShapePrimitive').
            



% Check tasks that belong to an object. 
task(O,T) :- triple(O, _, D), 
                triple(D, _, pap:'hasTask'), 
                triple(D, _, T),
                subclass_of(T,pap:'Task'). 

% Check which grasp regions belong to a task. 
region(T,R) :- subclass_of(T, D),
                    triple(D, _, pap:'GraspRegion'),        % 
                    triple(D,_, R),
                    subclass_of(R, pap:'Region').           % Grasp region has to be part of Region

% Combination task and region to query grasp region.
goal(O,T,R) :- region(T,R), 
                    task(O,T).





% Get multiple regions or test if this works...
% subclass_of(pap:'Pour', D), triple(D, W, pap:'GraspRegion'), triple(D,U, R), subclass_of(R, pap:'Region').

% Output of Region(pap:'Pour', Region)
% R: http://www.semanticweb.org/stanz/thesis-ontol#Back,
% U: http://www.w3.org/2002/07/owl#someValuesFrom,
% D: http://www.semanticweb.org/stanz/thesis-ontol##_:Description1,
% W: http://www.w3.org/2002/07/owl#onProperty.
         


% perishable(X) :- instance_of(X, pap:'Perishable').
% nonperishable(X) :- instance_of(X, pap:'NonPerishable').
% location(X,L) :- triple(X, dul:'hasLocation', L).


% shape(P,S) :- triple(P, dul:'hasShapeRegion', S).
% goal(G, S) :- triple(S, dul:'hasGoalPose', G).
% affordance(G,S) :- triple(S, dul'hasAffordance', G).



% grasp_location(A,P) :- triple(P, pap:'hasAffordance', A).
% mass(M, P) :- triple(P, soma:'hasMassAttribute', M).
% weight(M,W) :- triple(M, soma:'hasMassValue', W).


% result(A,P,W) :- grasp_location(A,P),   % retrieve affordance location.
%                 mass(M,P),              % check if object has mass attribute.
%                 weight(M,W),            % retrieve weight object.
%                 W < 2.                  % Weight of the object has to be below this value.

% Start simple! 
% grasp(P, T, A) :- triple(P, soma:'hasTask', T), 
%                 triple(T, soma:'hasConstraint', A)


% Hammer can be grasped from top and side
% Milk can be grasped from top and side. 

% Difficult approach
% grasp(P,T,A) :- triple(P, soma:'hasTask', T), 
%                 triple(T, soma:'hasRegion'Q),
%                 triple(Q, soma:'hasTask' T),
%                 triple(Q, soma:'hasRegionValue', C),
%                 triple(C, soma:'hasRelationwith', P),
%                 /- triple(P, soma:'hasRestrictions', P)
            


% add_elem_in_list(L, E, R) :- append(L, [E], R).


% % Add generated primitives to a list
% add_primitives(Primitives) :- 
%     shape_classification(Primitive, Shape), 
%     prim_list(Primitives, List), 
%     plan(P, []). 

% % New list with shapes for each primitive
% shape_rectangular([Eps1, Eps2], ListShape) :- 
%     (Eps1 * 1.5 - Eps2 < 0); 
%     (Eps1 * 0.5 + Eps2 > 0);
%     ListShape[L|"rectangular"].

% % New list with shapes for each primitive
% shape_cylindrical([Eps1, Eps2], ListShape) :- 
%     (Eps1 * 1.5 - Eps2 < 0); 
%     (Eps1 * 0.5 + Eps2 > 0);
%     ListShape[L|"rectangular"].

% % New list with shapes for each primitive
% shape_spherical([Eps1, Eps2], ListShape) :- 
%     (Eps1 * 1.5 - Eps2 < 0); 
%     (Eps1 * 0.5 + Eps2 > 0);
%     ListShape[L|"rectangular"].


% validate_cap(R, object ,shape, [SX, SY, SZ]) :-     % Returns a list with the classified superquadric
%     triple(Cap, hasShape, shape),                   % Check if the detected shape matches
%     triple(object, hasShape, shape),                % Check if the object has a shape like this
%     triple(Cap, hasRestriction, [Tmin, Tmax),       % Limitations of this shape
%     (Tmin < SX < Tmax; Tmin < SY < Tmax; Tmin < SZ < Tmax;), % Validation of it belongs to this shape
%     /+bottom_positioned([SZ], primitives)!          % if the cap is positioned at the bottom no grasp is valid
%     add_elem_in_list(List, Cap, R).                 % Add shape to the list if it belongs

% validate_handle(R, object, shape, [SX, SY, SZ]) :-
%     triple(Handle, hasShape, shape), 
%     triple(object, hasShape, shape), 
%     triple(Handle, hasRestriction, [Tmin, Tmax]), 
%     (Tmin < SX < Tmax; Tmin < SY < Tmax; Tmin < SZ < Tmax;), % Validation of it belongs to this shape    
%     add_elem_in_list(List, Cap, R).

% validate_containment(R, object, shape, [SX, SY, SZ]) :-
%     triple(Containment, hasShape, shape), 
%     triple(Object, hasShape, Shape), 
%     triple(Containment, hasRestriction, [Tmin, Tmax]), 
%     (Tmin < SX < Tmax; Tmin < SY < Tmax; Tmin < SZ < Tmax;), % Validation of it belongs to this shape    
%     add_elem_in_list(List, Cap, R). 

% validate_body(R, Object, Shape, [SX, SY, SZ]) :- 
%     triple(Containment, hasShape, shape), 
%     triple(Object, hasShape, Shape), 
%     triple(Containment, hasRestriction, [Tmin, Tmax]), 
%     (Tmin < SX < Tmax; Tmin < SY < Tmax; Tmin < SZ < Tmax;), % Validation of it belongs to this shape    
%     add_elem_in_list(List, Cap, R). 

% check_task(Prim, Object, Task, R) :- 
%     triple(Task, hasTasks, Tasks),              % check if the task exists
%     triple(Object, hasPrimitives, Primitive),   % which primitives belong to an object
%     triple(Object, hasTask, Task),              % check if the object affords the task
%     triple(Task, hasRestrictions, R),           % check the restrictions of the task
%     triple(R, hasGraspArea, G),                 % check which areas are allowed to grasp
%     triple(G, NotPartOf, Primitive),            % check if the unsuited grasp is not part of primitive. 
%     select_primitive(R, [X|_], Prim).           % select the primitive that best fits. 
%     plan(Primitive, Plan).

% check_valid_grasp(Object, Primitive, [X, Y, Z]) :- 

% % reason if no feasible grasp is found
% bottom_positioned(Primitives, Primitive) :- 
%     findall(primitive[z] < primitives[z]).

% top_positioned(Primitives, Primitive) :- 
%     findall(primitive[z] > primitives[z]).

% % if one area is not feasible, grasp other direction first
% temporarily_grasp(Result) :- 
%     Result == not_feasible,                     % check if grasp is not feasible
%     check_other_primitives([A], primitive),     % select other graspable primitive
%     Append(primitive),                          % append grasp to grasp_list
%     plan(Primtive, Plan).                       % add primitive to the plan

% % return list with grasps on the object
% Steps(ListTasks) :-
%     list([cylinder1, topgrasp], [rectangular, horizontalgrasp])


% % Update state of the product to being removed. So the knowledge keeps track of which products are where. 
% Start_grasp(Task) :- 
%     rdf_retract(Primitive), 
%     rdf_assert(Processing).

% % addition to check whether to close objects have a relation
% relation_objects(ObjectA, ObjectB, [PX1,PY1, PZ1], [PX2, PY2, PZ2]) :- 
%     sqrt(PX1-PX2, PY1-PY2, PZ1-PZ2, Dist), 
%     Dist < Treshold, 
%     pose_error_quatient(Q),
