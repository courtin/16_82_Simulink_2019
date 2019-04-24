function [] = draw_wing(root_LE, break_LE, tip_LE, c_tip, c_root, vortex, control, cs,ra,rb)
    f = figure();
    hold on 
    LE_semi_x = [root_LE(1),break_LE(1), tip_LE(1)];
    TE_semi_x = [root_LE(1)+c_root,break_LE(1)+c_root, tip_LE(1)+c_tip];
    LE_semi_y = [root_LE(2),break_LE(2), tip_LE(2)];
    LE_semi_z = [root_LE(3),break_LE(3), tip_LE(3)];
    LE_x = [fliplr(LE_semi_x), LE_semi_x];
    LE_y = [fliplr(-LE_semi_y), LE_semi_y];
    LE_z = [fliplr(LE_semi_z), LE_semi_z];
    TE_x = [fliplr(TE_semi_x), TE_semi_x];
    TE_y = LE_y;
    TE_z = LE_z;
    
    scatter3(vortex(2,:),vortex(1,:),vortex(3,:))
    scatter3(control(2,:),control(1,:),control(3,:))
    plot3(LE_y, LE_x, LE_z, 'k') %LE 
    plot3(TE_y, TE_x, TE_z, 'k') %TE
    plot3([tip_LE(2), tip_LE(2)], [tip_LE(1), tip_LE(1)+c_tip], [tip_LE(3), tip_LE(3)], 'k');%Right tip
    plot3([-tip_LE(2), -tip_LE(2)], [tip_LE(1), tip_LE(1)+c_tip], [tip_LE(3), tip_LE(3)], 'k');%Left tip
    
    plot3([break_LE(2), break_LE(2)], [break_LE(1), break_LE(1)+c_root], [break_LE(3), break_LE(3)], 'k--');%Right break
    plot3([-break_LE(2), -break_LE(2)], [break_LE(1), break_LE(1)+c_root], [break_LE(3), break_LE(3)], 'k--');%Left break
    
    for i = 1:length(vortex)
       %plot3([vortex(2,i), vortex(2,i)],[vortex(1,i)-cs(i)/4, vortex(1,i)+3*cs(i)/4], [vortex(3,i), vortex(3,i)], 'b--')
       if i > 1
           c = .5*(cs(i)+cs(i-1));
       else
           c = cs(i);
       end
       plot3([ra(2,i), ra(2,i)],[ra(1,i)-c/4, ra(1,i)+3*c/4], [ra(3,i), ra(3,i)], 'b--')
    end
    axis equal
    title("Wing bound-leg and control point locations")
    xlabel('y (ft)')
    ylabel('x (ft)')
    saveas(f,"wing.pdf")
end