function [moti_billi] = walk(moti_billi,distance,h)
%h = figure('Position', [10 10 1000 1000]);


%% define step specs
stepLength = 3; % max range of foot

%% define walk parameters
t_step = 1;     % 1 second for 1 footstep
num_steps = distance/stepLength;
num_its = 20;   % no of substeps in one footstep
c_step = t_step/num_its;

%% describing walk motion
for i = 1:num_steps
    % first half
    init_feet = [moti_billi.frontRight.endPoint, moti_billi.frontLeft.endPoint, moti_billi.backRight.endPoint, moti_billi.backLeft.endPoint]; 
    init_com = moti_billi.com;
    for t = linspace(0, t_step, num_its)

        x_foot = 0.5*(t/t_step*2*pi - sin(t/t_step*2*pi))*cos(moti_billi.zero_angle);
        y_foot = 0.5*(t/t_step*2*pi - sin(t/t_step*2*pi))*sin(moti_billi.zero_angle);
        z_foot = 0.5*(1 - cos(t/t_step*2*pi));
                   
        x_com = (x_foot/2);
        %y_com = (-(moti_billi.body.breadth/moti_billi.body.length)*x_com);
        y_com = (y_foot/2);
        z_com = 0;
        
        moti_billi = moti_billi.update( init_feet(:,1) + [x_foot; y_foot; z_foot],... % location of fr.endPoint
                                        init_feet(:,2),...                            % location of fl.endPoint
                                        init_feet(:,3),...                            % location of br.endPoint
                                        init_feet(:,4) + [x_foot; y_foot; z_foot],... % location of bl.endPoint
                                        init_com + [x_com; y_com; z_com],...          % location of CoM
                                        moti_billi.zero_angle);
        plotBody(moti_billi, h, false);
        pause(c_step);
    end
    
    % second half
    init_feet = [moti_billi.frontRight.endPoint, moti_billi.frontLeft.endPoint, moti_billi.backRight.endPoint, moti_billi.backLeft.endPoint]; 
    init_com = moti_billi.com;
    for t = linspace(0, t_step, num_its) 

        x_foot = 0.5*(t/t_step*2*pi - sin(t/t_step*2*pi))*cos(moti_billi.zero_angle);
        y_foot = 0.5*(t/t_step*2*pi - sin(t/t_step*2*pi))*sin(moti_billi.zero_angle);
        z_foot = 0.5*(1 - cos(t/t_step*2*pi));

        x_com = (x_foot/2);
        %y_com = (-(moti_billi.body.breadth/moti_billi.body.length)*x_com);
        y_com = (y_foot/2);
        z_com = 0;
        
        moti_billi = moti_billi.update( init_feet(:,1),...                            % location of fr.endPoint
                                        init_feet(:,2) + [x_foot; y_foot; z_foot],... % location of fl.endPoint
                                        init_feet(:,3) + [x_foot; y_foot; z_foot],... % location of br.endPoint
                                        init_feet(:,4),...                            % location of bl.endPoint
                                        init_com + [x_com; y_com; z_com],...          % location of CoM
                                        moti_billi.zero_angle);
        plotBody(moti_billi, h, false);
        pause(c_step);
    end
end