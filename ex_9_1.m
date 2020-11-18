%Exercise 9_1: Position Control of a Two-Link Manipulator
%Ref: Book "Applied Nonlinear Control", p.396 , Author: J. J Slotine.

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
m1 = 1;
L1 = 1;
me = 2;
delta_e = 30;
I1 = 0.12;
Icl = 0.5;
Ie = 0.25;
Ice = 0.6;

a1 = I1 + m1 * Icl^2 + Ie + me * Ice^2 + me * L1^2;
a2 = Ie + me * Ice^2;
a3 = me * L1 * Ice * cos(delta_e);
a4 = me * L1 * Ice * sin(delta_e);
