function [] = draw_wing2(sections, vortex, cs,ra,rb, vortex_cs)
    f = figure();
    hold on 
    LE_semi_x = sections(1,:);
    TE_semi_x = sections(1,:)+cs;
    LE_semi_y = sections(2,:);
    LE_semi_z = sections(3,:);
    %LE_x = [fliplr(LE_semi_x), LE_semi_x];
    %LE_y = [fliplr(-LE_semi_y), LE_semi_y];
    %LE_z = [fliplr(LE_semi_z), LE_semi_z];
    %TE_x = [fliplr(TE_semi_x), TE_semi_x];
    LE_x = LE_semi_x;
    TE_x = TE_semi_x;
    LE_y = LE_semi_y;
    LE_z = LE_semi_x;
    TE_y = LE_y;
    TE_z = LE_z;
    
    scatter3(vortex(2,:),vortex(1,:),vortex(3,:))
    plot3(LE_y, LE_x, LE_z, 'k') %LE 
    plot3(TE_y, TE_x, TE_z, 'k') %TE
    plot3([sections(2,end), sections(2,end)], [sections(1,end), sections(1,end)+cs(end)], [sections(3,end), sections(3,end)], 'k');%Right tip
    plot3([-sections(2,end), -sections(2,end)], [sections(1,end), sections(1,end)+cs(end)], [sections(3,end), sections(3,end)], 'k');%Left tip
       
    for i = 1:length(vortex(1,:))
       %plot3([vortex(2,i), vortex(2,i)],[vortex(1,i)-cs(i)/4, vortex(1,i)+3*cs(i)/4], [vortex(3,i), vortex(3,i)], 'b--')
       c = vortex_cs(i);
       plot3([ra(2,i), ra(2,i)],[ra(1,i), ra(1,i)+c], [ra(3,i), ra(3,i)], 'b--')
    end
    axis equal
    title("Wing bound-leg and control point locations")
    xlabel('y (ft)')
    ylabel('x (ft)')
    saveas(f,"wing.pdf")
end