%Exercise 9_2: Sliding Control of a Two-Link Manipulator's Position
%Ref: Book "Applied Nonlinear Control", p.403 , Author: J. J Slotine.

%The Dynamics of a simple robotic manipulator can be written as:
%H(q)ddot(q) + C(q,dot(q))dot(q) + G(q) = Tau
%where...
%H(q) = inertia matrix (symmetric, positive definite);
%C(q,dot(q))dot(q) = Centripetal and Coriolis torques
%G(q) = gravitational torques

%Consider a planar, two-link, articulated manipulator.
%whose position can be described by a 2-vector q of joint angles,
%and whose actuator inputs consist of a 2-vector z of torques 
%applied at the manipulator joints.

%given parameters
m1 = 1; % 0.8 < m1 < 1.3
L1 = 1;
me = 2; %1.6 < me < 2.6
delta_e = 0.523599; %30 degree in rad
I1 = 0.12;
Icl = 0.5;
Ie = 0.25;
Ice = 0.6;

me_hat = 2;
ml_hat = 1;

a1 = I1 + ml_hat * Icl^2 + Ie + me_hat * Ice^2 + me_hat * L1^2;
a2 = Ie + me_hat * Ice^2;
a3 = me_hat * L1 * Ice * cos(delta_e);
a4 = me_hat * L1 * Ice * sin(delta_e);
