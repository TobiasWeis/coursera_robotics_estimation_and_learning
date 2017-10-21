function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y
    % Check if the first time running this function
    t-previous_t
    if previous_t<0      
        %dt = 1/33.0;
        dt = 0.0335;
        param.A = [ [1,0,dt,0];
                    [0,1,0,dt];
                    [0,0,1,0];
                    [0,0,0,1]];    
        
        state = [x, y, 0, 0]';
        
        param.C = [ [1,0,0,0];
                    [0,1,0,0]];
                
        param.P = diag([0.1,0.1,0.01,0.01]);
    
        sigx = 0.1;
        sigy = 0.1;
        param.R = diag([0,0,sigx^2,sigy^2]);
        
        sigzx = 0.1;
        sigzy = 0.1;
        param.Q = diag([sigzx^2, sigzy^2]);
        
        predictx = x;
        predicty = y;
    end
    
    % prediction
    statepred = param.A*state
    Ppred = param.A*param.P*param.A' + param.R;
    
    % correction
    K = Ppred*param.C'*inv(param.C*Ppred*param.C' + param.Q);
    [x,y]' - param.C*statepred;
    
    state = statepred + K*([x,y]' - param.C*statepred);
    param.P = (eye(4) - K*param.C)*Ppred;
    
    if previous_t>0
        predictx = statepred(1);
        predicty = statepred(2);
    end
end
