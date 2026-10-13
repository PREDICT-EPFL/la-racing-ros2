function [fx, fy, kappa, n_point, data_points] = la_track_spline_fit()

% part 1: given points to go through and arc length between consecutive points, 
% use matlab spline function to get the polynomial coefficients 

data_points = [1,    0; % slope of first point
               0,    0; % first point (x,y)
               0.5,  0;
               1,    0;
               1.75, 0;
               2.25, 0.5;
               1.75, 1;
               1.25, 1;
               0.75, 1;
               0.25, 1.5;
               0.25, 2;
               -0.25, 2.5;
               -0.75, 2.5;
               -1.25, 2;
               -1.25, 1.5;
               -1.25, 1;
               -1.25, 0.5;
               -0.75, 0;
               0,     0; % last point, should be the same as first
               1,     0]'; % slope of last point 
[~, n_point] = size(data_points);
n_point = n_point - 2;
distances = [0;
             0.5; 
             1;
             1.75;
             1.75+pi/4;
             1.75+pi/2; 
             2.25+pi/2;
             2.75+pi/2; 
             2.75+3*pi/4; 
             3.25+3*pi/4; 
             3.25+pi;
             3.75+pi; 
             3.75+5*pi/4; 
             4.25+5*pi/4;
             4.75+5*pi/4;
             5.25+5*pi/4; 
             5.25+3*pi/2; 
             6+3*pi/2];
la_track = spline(distances, data_points);
track_length = distances(end);
dist_grid = linspace(0, 1, 100);

% part 2: use the polynomial coefficients to construct casadi function

import casadi.*
la_track_breaks = MX(la_track.breaks);              % L + 1
la_track_coefs = MX(la_track.coefs);                % (2xL) x 4
v = MX.sym('v');                                    % scalar input, range [0, \infty). v=0.x or v=1.x returns the same. The integer part = laps, the decimal part = current progress. 
v_decimal = v - floor(v);                           % get the decimal part 
v_reg = v_decimal * track_length;                   % regularize to [0, track_length]
lowest_index = low(la_track_breaks, v_reg) + 1;     % the most 
tmp = v_reg - la_track_breaks(lowest_index);
basis = [tmp^3, tmp^2, tmp, 1];
coeff_x = la_track_coefs(2*lowest_index-1,:);
coeff_y = la_track_coefs(2*lowest_index,:);
pos_x = dot(coeff_x, basis);
pos_y = dot(coeff_y, basis);

fx = Function('fx', {v}, {pos_x});
fy = Function('fy', {v}, {pos_y});

pos_x_grad = gradient(pos_x, v);
pos_y_grad = gradient(pos_y, v);

fx_gradient = Function('fx_gradient', {v}, {pos_x_grad});
fy_gradient = Function('fy_gradient', {v}, {pos_y_grad});

pos_x_hess = gradient(pos_x_grad, v);
pos_y_hess = gradient(pos_y_grad, v);

fx_hessian = Function('fx_hessian', {v}, {pos_x_hess});
fy_hessian = Function('fy_hessian', {v}, {pos_y_hess});

k = abs((pos_x_grad*pos_y_hess - pos_y_grad*pos_x_hess)/((pos_x_grad^2+pos_y_grad^2)^1.5));
kappa = Function('curvature', {v}, {k});


figure()
hold on
plot(full(fx(dist_grid)), full(fy(dist_grid)), '-b')
plot(data_points(1,2:n_point),data_points(2,2:n_point),'or')
axis equal

% plot gradients 
% figure()
% title('gradient')
% hold on 
% plot(dist_grid, full(fx_gradient(dist_grid)),'r')
% plot(dist_grid, full(fy_gradient(dist_grid)),'b')


% plot hessian 
% figure()
% title('hessian')
% hold on 
% plot(dist_grid, full(fx_hessian(dist_grid)),'r')
% plot(dist_grid, full(fy_hessian(dist_grid)),'b')

% plot curvature 
figure()
title('curvature')
hold on 
plot(dist_grid, full(kappa(dist_grid)),'r')

end


% fx.generate('fx.c');
% fy.generate('fy.c');
% 
% Cx = CodeGenerator('fx.c');
% Cx.add(fx);
% Cx.generate();
% 
% Cy = CodeGenerator('fy.c');
% Cy.add(fy);
% Cy.generate();