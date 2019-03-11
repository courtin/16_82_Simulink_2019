%[CL_coeffs,CD_coeffs,CM_coeffs]=get_regression_coefficients(MAT,10,2,2,2);

MAT = process_data();

alphas=[-5:1:25]';
deltas=[0 20 40 55 90].*ones(length(alphas),1);
CJs=[0:0.5:4.5].*ones(length(alphas),1);
%% make the plotting colors

plottingcolor = linspace(0,1, size(CJs,2))';
colormap_hot=zeros(100,3);
colormap_hot(:,1) = linspace(0,1,100)';

%% plots
ha = tight_subplot(2,5,[.04 .04],[.1 .01],[.04 .04]);
for k=[10,20]
%     for k=[20]
for i=1:size(deltas,2)
% for i=1
    %delta_check=MAT(:,2)==deltas(1,i) & MAT(:,8)==k;
    delta_check=MAT(:,8)==k;
    delta_check2=MAT(:,2)==deltas(1,i) & MAT(:,8)==k;
%     alpha_min=min(MAT(delta_check2,1));
%     alpha_max=max(MAT(delta_check2,1));
    alpha_min=-30;
    alpha_max=30;
    if abs(alpha_max)>0
        alphas=[alpha_min:0.1:alpha_max]';
        deltas=[0 20 40 55 90].*ones(length(alphas),1);
        CJs=[0:0.5:4.5].*ones(length(alphas),1);
    end
    [CL_coeffs,CD_coeffs,CM_coeffs]=get_regression_coefficients(MAT(delta_check,:),k,1,1,1);
%     subplot(2,5,i+(k-10)/2)
    caxis([0 4.5])
    c=jet(size(CJs,2));
    axes(ha(i+(k-10)/2))
    %@Alex this is where it makes the plots for the individual delta_f
    %values.
    for j=1:size(CJs,2)
        [CL,CD,CM]=regression_results(alphas, deltas(:,i), CJs(:,j),CL_coeffs,CD_coeffs,CM_coeffs);
        plot(CD,CL,'-','DisplayName',strcat('\Delta C_J = ',num2str(CJs(1,j))),'LineWidth', 1.5,'Color',[plottingcolor(j),0,0])
%         plot(alphas,CD,'-','DisplayName',strcat('\Delta C_J = ',num2str(CJs(1,j))),'LineWidth', 1)
        hold on
        
    end
    check2=MAT(:,8)==k & MAT(:,2)==deltas(1,i);
    scatter(MAT(check2, 6),MAT(check2, 5), 100,MAT(check2,4), '.')

    colormap(colormap_hot)
    caxis([0 4.5])
    title(strcat('\delta_f=', num2str(deltas(1,i)),', \delta_M=', num2str(k)))
    grid on
    xlabel('$$\it c_x $$','Interpreter','latex','fontsize',18)
    ylabel('$$\it c_\ell $$','Interpreter','latex','fontsize',18)
    xlim([-6 3])
    ylim([-2 10])
%     set(gca,'YTickLabel',[])
%     set(gca,'XTickLabel',[])
    axis square
    set(gca,'FontSize',17)
%     axis([-5 1 -2 6])
    hold off
end
end

% figure
% colormap(colormap_hot)
% caxis([0 4])
% colorbar
% caxis([0 4])

