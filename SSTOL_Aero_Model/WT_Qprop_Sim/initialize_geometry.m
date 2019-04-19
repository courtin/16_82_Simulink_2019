function [vortex, cs, vg_dCJ1, vg_flap, N, bref, Sref, vortex_cs, ra, rb] = initialize_geometry(verbose, g_flap_in, g_dCJ1_in)

%Define sections on one wing half-span
 g_M1 = g_dCJ1_in(1);
 g_M2 = g_dCJ1_in(2);
 g_M3 = g_dCJ1_in(3);
 g_M4 = g_dCJ1_in(4);
 g_M5 = g_dCJ1_in(5);
 g_M6 = g_dCJ1_in(6);
 g_M7 = g_dCJ1_in(7);
 g_M8 = g_dCJ1_in(8);

 g_F1 = g_flap_in(1);
 g_F2 = g_flap_in(2);
 g_F3 = g_flap_in(3);
 g_F4 = g_flap_in(4);

%               Root    fuse    P1      P2      P3      Break   P4      Tip      
sections = [    0       0       0       0       0       0       .126    .42;
                0       .75     1.8     2.9     4       4.5     5.1     6.5;
                0       0       0       0       0       0       0       0];
sections = make_sym(sections,2);            
sections = sections(:,[1:8,10:16]);

%cs      =  [    1.5     1.5     1.5     1.5     1.5     1.5     1.176   .92];
cs      =  [    1.5     1.5     1.5     1.5     1.5     1.5     1.324   .92];
cs = make_sym(cs);
cs = cs(:,[1:8,10:16]);

Ns      =       [    3       3       3       3       1      1       3];
Ns      = make_sym(Ns);
N       = sum(Ns);

%               Tip_>p4     P4->B   B->P3   P3->P2  P2->P1 P1->F   Fuse -> root
g_flap  =       [   0        0      g_F1    g_F1    g_F2   g_F2    g_F2...
                    g_F3    g_F3    g_F3   g_F4    g_F4    0          0];
                
g_dCJ1  =       [  0        g_M1    g_M1   g_M2    g_M3    g_M4       g_M4 ...
                    g_M5    g_M5    g_M6   g_M7    g_M8    g_M8       0];




Sref = 17.95;
bref = 13;


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
    l_gf    = zeros(1,Ns(j));
    l_gcj   = zeros(1,Ns(j));
    
    for k = 1:Ns(j)
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
    if j < M/2
        vortex_gflap(start:stop-1) = l_gf;
        vortex_dCJ1(start:stop-1)  = l_gcj;
    else
        vortex_gflap(start+1:stop) = l_gf;
        vortex_dCJ1(start+1:stop)  = l_gcj;
    end
end
%vortex_cs = make_sym(fliplr(vortex_cs(1:35)))
vortex_gflap(36) = vortex_gflap(35); 
vortex_dCJ1(36) = vortex_dCJ1(35);

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
%ra_sym(2,:) = ra_sym(2,:)*-1;
%rb_sym(2,:) = rb_sym(2,:)*-1;
%rb = [fliplr(ra_sym), rb];
%ra = [fliplr(rb_sym), ra];

%vortex = make_sym(vortex,2);
%dCJs = make_sym(dCJs);
%cs = make_sym(cs);
%vortex_cs = make_sym(vortex_cs);
%vg_flap = make_sym(vg_flap);
%vg_dCJ1 = make_sym(vg_dCJ1);

%dvortex = [fliplr(dvortex), dvortex];
dy = rb(2,:)-ra(2,:);
%sections = make_sym(sections,2);
if verbose
    draw_wing2(sections, vortex, cs,ra,rb, vortex_cs)
end


end

