function [vortex, cs, vg_dCJ1, vg_flap, N, bref, Sref, vortex_cs, ra, rb] = initialize_geometry(verbose)

%Define sections on one wing half-span
g_M1 = 1.25;
g_M2 = 1.25;
g_M3 = 1.25;
g_M4 = 0;
g_F1 = 1;
g_F2 = 1;

%               Root    fuse    P1      P2      P3      Break   P4      Tip      
sections = [    0       0       0       0       0       0       .126    .42;
                0       .75     1.8     2.9     4       4.5     5.1     6.5;
                0       0       0       0       0       0       0       0];
            
%cs      =  [    1.5     1.5     1.5     1.5     1.5     1.5     1.176   .92];
cs      =  [    1.5     1.5     1.5     1.5     1.5     1.5     1.324   .92];

g_flap  =       [  g_F1    g_F1    g_F1   g_F2    g_F2    0          0];
g_dCJ1  =       [  g_M1    g_M1    g_M2   g_M3    g_M4    g_M4       0];

% g_flap  =       [    1      1       1       1       1       0       0];
% g_dCJ1  =       [    1      1       1       1       1       1       0];

Ns      =       [    5       5       5       5       5       5       5];

Sref = 17.95;
bref = 13;

N = sum(Ns);
lines = zeros(3,N+1);
M = length(sections(1,:));
vortex_cs = zeros(1, N+1);
vortex_gflap = zeros(1, N+1);
vortex_dCJ1  = zeros(1, N+1);
for j = 1:M-1
    start = sum(Ns(1:j-1))+1;
    stop = start+Ns(j);
    lines(:,start:stop) = cosspace([sections(:,j),sections(:,j+1)], Ns(j)+1);
    
    l_cs    = zeros(1,Ns(j)+1);
    l_gf    = zeros(1,Ns(j)+1);
    l_gcj   = zeros(1,Ns(j)+1);
    
    for k = 1:Ns(j)+1
        c_root = cs(j);
        c_tip = cs(j+1);
        y = lines(2,start+k-1);
        yb = lines(2,start);
        yt = lines(2,stop);
        l_cs(k)      = c_root - (c_root-c_tip)*(y-yb)/(yt-yb);
        %l_gf(k)     = interp1([yb,yt], [g_flap(j), g_flap(j+1)], y);
        l_gf(k)     = g_flap(j);
        %l_gcj(k)    = interp1([yb,yt], [g_dCJ1(j), g_dCJ1(j+1)], y);
        l_gcj(k)    = g_dCJ1(j);
        %lcs(k) = cs(j) - (cs(j)-cs(j-1))*(lines(2,start+k-1)-lines(2,start))/(lines(2,stop)-lines(2,start));
    end
        
    vortex_cs(start:stop)    = l_cs;
    vortex_gflap(start:stop) = l_gf;
    vortex_dCJ1(start:stop)  = l_gcj;
end

ra = lines(:,1:N);
rb = lines(:,2:N+1);
vortex_cs(end) = cs(end);
vortex_gflap(end) = g_flap(end);
vortex_dCJ1(end)  = g_dCJ1(end);

[vortex, dvortex] = midpoints(lines);
vortex_cs       = midpoints(vortex_cs);
vg_flap         = midpoints(vortex_gflap);
vg_dCJ1         = midpoints(vortex_dCJ1);
vortex(1,:)     = vortex(1,:) + vortex_cs/4;
%Add other half of wing, symmetric around y-axis
ra_sym = ra;
rb_sym = rb;
ra_sym(2,:) = ra_sym(2,:)*-1;
rb_sym(2,:) = rb_sym(2,:)*-1;
rb = [fliplr(ra_sym), rb];
ra = [fliplr(rb_sym), ra];

vortex = make_sym(vortex,2);
%dCJs = make_sym(dCJs);
cs = make_sym(cs);
vortex_cs = make_sym(vortex_cs);
vg_flap = make_sym(vg_flap);
vg_dCJ1 = make_sym(vg_dCJ1);

dvortex = [fliplr(dvortex), dvortex];
dy = rb(2,:)-ra(2,:);
sections = make_sym(sections,2);
if verbose
    draw_wing2(sections, vortex, cs,ra,rb, vortex_cs)
end


end

