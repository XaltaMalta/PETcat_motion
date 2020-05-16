%% define billi specs
link1 = 3;      % linkLength1 
link2 = 4;      % linkLength2

length = 10;    % length of body
width  = 8;     % width of body

orientation_matrix = [1 0 0 0 ; 0 1 0 0; 0 0 1 0; 0 0 0 1];     % orientation of body
body_obj = Body(length ,width , orientation_matrix);            % declare body object

moti_billi = Billi(body_obj,link1,link2);                       % declare Billi object

% initial pose of quad
com = [0; 0; 6];
moti_billi = moti_billi.update( [5*1.414*cos(-pi/4) ; 5*1.414*sin(-pi/4) ; -com(3)] + com,...        % location of fr.endPoint
                                [5*1.414*cos(pi/4)  ; 5*1.414*sin(pi/4)  ; -com(3)] + com,...        % location of fl.endPoint
                                [-5*1.414*cos(pi/4) ; -5*1.414*sin(pi/4) ; -com(3)] + com,...        % location of br.endPoint
                                [-5*1.414*cos(-pi/4); -5*1.414*sin(-pi/4); -com(3)] + com,...        % location of bl.endPoint
                                com,...                                                              % location of CoM
                                0);
                            
list = [5,10;5,-20];
moti_billi = locomote(moti_billi,list);