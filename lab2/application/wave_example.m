c = 1;
a = 0;
f = 0;
m = 1;

numberOfPDE = 1;
model = createpde(numberOfPDE);
geometryFromEdges(model,@squareg);
pdegplot(model,'EdgeLabels','on'); 
ylim([-1.1 1.1]);
axis equal
title 'Geometry With Edge Labels Displayed';
xlabel x
ylabel y

specifyCoefficients(model,'m',m,'d',0,'c',c,'a',a,'f',f);

%applyBoundaryCondition(model,'dirichlet','Edge', 1,'u',0);

generateMesh(model);
figure
pdemesh(model);
ylim([-1.1 1.1]);
axis equal
xlabel x
ylabel y

%u0 = @(location) atan(cos(pi/2*location.x));
%ut0 = @(location) 3*sin(pi*location.x).*exp(sin(pi/2*location.y));
u0 = @(location) 1;
%ut0 = @(location) 0;
setInitialConditions(model,u0,ut0);

n = 31;
tlist = linspace(0,5,n);

model.SolverOptions.ReportStatistics ='on';
result = solvepde(model,tlist);

u = result.NodalSolution;

figure
umax = max(max(u));
umin = min(min(u));
for i = 1:n
    pdeplot(model,'XYData',u(:,i),'ZData',u(:,i),'ZStyle','continuous',...
                  'Mesh','off','XYGrid','on','ColorBar','off');
    axis([-1 1 -1 1 umin umax]); 
    caxis([umin umax]);
    xlabel x
    ylabel y
    zlabel u
    M(i) = getframe;
end