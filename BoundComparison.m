%%  Heuristic Comparison
%   Avery Bodenstein
%   11/27/22

%%  Inputs
clc; clear; close all

nPoints = 1000;
n = logspace(0,3,nPoints);
epsilon = linspace(0,1,nPoints);

%%  Calculate bounds
MDG = harmonic(n);
MDGa = log(n) + 0.57;
GIC = sqrt(n)./2;
DFS = 2*ones(size(n));
EDa = min(2,1./(1-sqrt(1-epsilon)));
EDm = min(2,1./epsilon);
LL = sqrt(n)/2 + 3/2;
LR = n;

%%  Plots
figure
semilogx(n,MDG)
hold on
% semilogx(n,MDGa,'--')
semilogx(n,GIC)
semilogx(n,DFS)
semilogx(n,EDa)
semilogx(n,EDm)
xlabel('Degree (Epsilon)')
ylabel('Bound')
grid minor
legend('Maximum Degree Greedy (MDG)','Greedy Independant Cover (GIC)','Depth First Search (DFS)','Edge Deletion (avg \epsilon 0.8)','Edge Deletion (min \epsilon 0.8)','location','best')


figure
subplot(3,1,1)
semilogx(n,MDG)
hold on
semilogx(n,GIC)
semilogx(n,LL)
% semilogx(n,LR)
xlabel('Maximimum Degree (\Delta)')
ylabel('Approximation Ratio')
grid minor
legend('Maximum Degree Greedy (MDG)','Greedy Independant Cover (GIC)','List Left (LL)','List Right (LR)','location','best')

subplot(3,1,2)
plot(epsilon,EDa)
hold on
plot(epsilon,EDm)
xlabel('Degree Ratio (\epsilon)')
grid minor
legend('Edge Deletion (average)','Edge Deletion (minimum)','location','best')

subplot(3,1,3)


