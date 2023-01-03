% example of Prolog module defined by the user
% STATUS: working

% module definition and public (externally visible) predicates
:- module(shape_to_task,
    [
        affordTask/3,
        primitive_values/5
    ]).



:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- dynamic primitive_values/5. 


:- rdf_meta
    affordTask(+,r,r),
    primitive_values(+,+,+,+,+).


affordTask(ID, Surface, pap:'Cook') :-
    primitive_values(ID_tool, _, Dim_tool, _,_),
    min_dimension(Dim_tool, [0.0, 0.08, 0.08]), 
    max_shape(ID_tool), 

    primitive_values(ID, _, Dim, _,Semantic_shape), 
    min_ratio(Dim, 1.5),   
    max_dimension(Dim, [0.05, 0.05, 1.00]), 
    min_dimension(Dim, [0.005, 0.01, 0.05]), 
    Semantic_shape == 'cylinder',
    ID \= ID_tool,
    Surface = 'Round'.

affordTask(ID, Surface, pap:'Hammer') :-
    primitive_values(ID_tool, _, _, _, _),
    % max_ratio(Dim_tool, 4), 

    primitive_values(ID, _, Dim, _, _),
    max_dimension(Dim, [0.07, 0.07, 1.00]), 
    min_dimension(Dim, [0.005, 0.01, 0.05]), 
    min_ratio(Dim, 1.5),   
    max_shape(ID), 
    ID_tool \= ID, 
    Surface = 'Round'.

affordTask(ID, Surface, pap:'Pour'):- 
    primitive_values(ID, _, Dim, _,Semantic_shape),  
    max_shape(ID),
    min_ratio(Dim, 1.5),
    max_dimension(Dim, [0.08, 0.08, 1.00]),
    min_dimension(Dim, [0.03, 0.03, 0.03]),
    Semantic_shape == 'cylinder',
    Surface = 'Round'. 
    
affordTask(ID, Surface, pap:'Scoop') :-
    primitive_values(ID_tool, _, Dim_tool, _,_),
    max_ratio(Dim_tool, 2), 
    min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

    primitive_values(ID, _, Dim, _, _),  
    max_dimension(Dim, [0.005, 0.03, 1.00]),
    min_dimension(Dim, [0.00, 0.00, 0.03]),
    ID \= ID_tool, 
    Surface = 'All'.

affordTask(ID, Surface, pap:'Turn') :- 
    primitive_values(ID_tool, _, Dim_tool, _,_),
    max_dimension(Dim_tool, [0.03, 0.1, 0.1]),
    min_dimension(Dim_tool, [0.00, 0.03, 0.03]),

    primitive_values(ID, _, Dim, _, Semantic_shape),
    min_ratio(Dim, 2), 
    max_dimension(Dim, [0.04, 0.04, 1.00]),
    min_dimension(Dim, [0.00, 0.00, 0.03]),
    (Semantic_shape == 'cylinder';
        Semantic_shape == 'cuboid'),
    ID \= ID_tool, 
    Surface = 'All'. 


affordTask(ID, Surface, pap:'HandOver') :-
    primitive_values(ID, _, _, _,_),
    min_shape(ID),
    Surface = 'All'.


affordTask(ID, Surface, pap:'Placement') :-
    primitive_values(ID, _, _, _,_),
    max_shape(ID),
    Surface = 'Flat'.


%%%%%%% ADDITIONAL DUMMY TASKS TO MEASURE THE PERFORMANCE %%%%%%%%%%%% 

% affordTask(ID, Surface, pap:'Scoop1') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop2') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.


% affordTask(ID, Surface, pap:'Scoop3') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop4') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop5') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   

% affordTask(ID, Surface, pap:'Scoop6') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop7') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.


% affordTask(ID, Surface, pap:'Scoop8') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop9') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop10') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   




% affordTask(ID, Surface, pap:'Scoop11') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop12') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.


% affordTask(ID, Surface, pap:'Scoop13') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop14') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop15') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   


% % 20 TOTAL 

% affordTask(ID, Surface, pap:'Scoop16') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop17') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop18') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop19') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop20') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   




% affordTask(ID, Surface, pap:'Scoop21') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop22') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.


% affordTask(ID, Surface, pap:'Scoop23') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop24') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop25') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   

% affordTask(ID, Surface, pap:'Scoop26') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop27') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.


% affordTask(ID, Surface, pap:'Scoop28') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop29') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop30') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.
% affordTask(ID, Surface, pap:'Scoop31') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.
% affordTask(ID, Surface, pap:'Scoop32') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.


% affordTask(ID, Surface, pap:'Scoop33') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop34') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop35') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   

% affordTask(ID, Surface, pap:'Scoop36') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop37') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.


% affordTask(ID, Surface, pap:'Scoop38') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop39') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop40') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.  
% affordTask(ID, Surface, pap:'Scoop41') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop42') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.


% affordTask(ID, Surface, pap:'Scoop43') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop44') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop45') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% % % 50 TOTAL
% affordTask(ID, Surface, pap:'Scoop46') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop47') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.


% affordTask(ID, Surface, pap:'Scoop48') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop49') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop50') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop51') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop52') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.


% affordTask(ID, Surface, pap:'Scoop53') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop54') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop55') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   

% affordTask(ID, Surface, pap:'Scoop56') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop57') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.


% affordTask(ID, Surface, pap:'Scoop58') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop59') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.

% affordTask(ID, Surface, pap:'Scoop60') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   


% affordTask(ID, Surface, pap:'Scoop61') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop62') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop63') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop64') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop65') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop66') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop67') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop68') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop69') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop70') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop71') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop72') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop73') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop74') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop75') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop76') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop77') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop78') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop79') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop80') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop81') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop82') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop83') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop84') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop85') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop86') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop87') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop88') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop89') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop90') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop91') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop92') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop93') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop94') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop95') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   


% % 100 Iterations
% affordTask(ID, Surface, pap:'Scoop96') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop97') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop98') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop99') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop100') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop101') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop102') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop103') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop104') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop105') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop106') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop107') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop108') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop109') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop110') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop111') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop112') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop113') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop114') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop115') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop116') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop117') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop118') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop119') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop120') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop121') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop122') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop123') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop124') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop125') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop126') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop127') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop128') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop129') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop130') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop131') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop132') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop133') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop134') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop135') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop136') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop137') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop138') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop139') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop140') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop141') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop142') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop143') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop144') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop145') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop146') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop147') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop148') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop149') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop140') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop141') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop142') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop143') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop144') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop145') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop146') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop147') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop148') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop149') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop150') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop151') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop152') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop153') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop154') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop155') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop156') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop157') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop158') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop159') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop160') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop161') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop162') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop163') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop164') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop165') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop166') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop167') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop168') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop169') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop170') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop171') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop172') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop173') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop174') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop175') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop176') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop177') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop178') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop179') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop180') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop181') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop182') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop183') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop184') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop185') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop186') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop187') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop188') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop189') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop190') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop191') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop192') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop193') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop194') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop195') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop196') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop197') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop198') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop199') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
% affordTask(ID, Surface, pap:'Scoop200') :-
%     primitive_values(ID_tool, _, Dim_tool, _,_),
%     max_ratio(Dim_tool, 2), 
%     min_dimension(Dim_tool, [0.01, 0.01, 0.01]), 

%     primitive_values(ID, _, Dim, _, _),  
%     max_dimension(Dim, [0.005, 0.03, 1.00]),
%     min_dimension(Dim, [0.00, 0.00, 0.03]),
%     ID \= ID_tool, 
%     Surface = 'All'.   
