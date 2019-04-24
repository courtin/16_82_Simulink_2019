function trajectory_reconstruction(stateHistory)

%stateHistory = results_state;

t = stateHistory.Time(1:end-1);

Xe = stateHistory.Data(1:end-1,10);
Ye = stateHistory.Data(1:end-1,11);
Ze = -stateHistory.Data(1:end-1,12);

if range(Ye) == 0
    %only longitudinal, so a 2D plot will suffice
    
    fig_h = figure
    hold on
    %mark start point
    scatter(Xe(1), Ze(1), 'go','filled')
    %mark end point
    scatter(Xe(end), Ze(end), 'ro','filled')
    %plot data
    plot(Xe,Ze,'k','LineWidth',3)
    scatter(Xe,Ze,[],t,'filled')
    colorbar
    grid on
    title(sprintf('Trajectory (2D) over %.1f seconds', max(t)))
    xlabel('X')
    ylabel('Z')
    ylim([min(min(Ze),0) max(Ze)+10])
    
else
    
    %% 3D reconstruction
    fig_h = figure
    hold on
    %mark start point
    scatter3(Xe(1), Ye(1), Ze(1), 'go','filled')
    %mark end point
    scatter3(Xe(end), Ye(end), Ze(end), 'ro','filled')
    
    %plot data
    plot3(Xe,Ye,Ze,'k','LineWidth',3)
    
    % reverse Y-axis so matches with left-handed simulation
    aH = gca;
    aH.YDir = 'reverse';
    
    %make it look nice
    grid
    view([-45,45])
    title(sprintf('Trajectory (3D) over %.1f seconds', max(t)))
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    
    %% generate grid
    xtick = xticks;
    ytick = yticks;
    ztick = zticks;
    
    xl = xlim;
    yl = ylim;
    zl = zlim;
    xmin = xl(1);
    xmax = xl(2);
    ymin = yl(1);
    ymax = yl(2);
    zmin = zl(1);
    zmax = zl(2);
    
    for i = 1:length(ytick)
        v = [xmin, ytick(i), zmin; xmax, ytick(i), zmin; xmax, ytick(i), zmax;xmin, ytick(i), zmax];
        f = [1 2 3 4];
        patch('Faces',f,'Vertices',v,'FaceColor','blue','FaceAlpha',0.03)
        
    end
    
    
    
end

end
