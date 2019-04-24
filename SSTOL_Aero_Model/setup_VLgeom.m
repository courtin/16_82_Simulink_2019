function [vortex, cs, N, bref, Sref, vortex_cs, ra, rb, sections, Ns, C_flap, C_motor] = setup_VLgeom(verbose)
%Reference quantities
Sref = 17.95;
bref = 13;

%Define sections on one wing half-span
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


%Create VL derived geometry
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
    
    for k = 1:Ns(j)
        c_root = cs(j);
        c_tip = cs(j+1);
        y = lines(2,start+k-1);
        yb = lines(2,start);
        yt = lines(2,stop);
        l_cs(k)      = c_root - (c_root-c_tip)*(y-yb)/(yt-yb);
    end
    vortex_cs(start:stop)    = l_cs;
end

ra = lines(:,1:N);
rb = lines(:,2:N+1);
vortex_cs(end) = cs(end);

[vortex, dvortex] = midpoints(lines);
vortex_cs       = midpoints(vortex_cs);
vortex(1,:)     = vortex(1,:) + vortex_cs/4;

C_flap  = zeros(N,4);  %Hard-coded for four flaps
C_motor = zeros(N,8);  %Hard-coded for eight motors


%               Tip_>p4     P4->B   B->P3   P3->P2  P2->P1 P1->F   Fuse -> root
g_flap  =       [  0         0      1       1        2     2       0 ...
                   0         3      3       4        4     0       0];
                
g_dCJ1  =       [  0         1      1       2        3     4       0 ...
                   0         5      6       7        8     8       0];

Ntot = cumsum(Ns);
start = [1, Ntot(1:end-1)+1];
ends = Ntot;
for i = 1:M-1
    if g_flap(i) > 0
        C_flap(start(i):ends(i),g_flap(i)) = 1;
    end
    if g_dCJ1(i) > 0
        C_motor(start(i):ends(i),g_dCJ1(i)) = 1;
    end
end
dy = rb(2,:)-ra(2,:);

if verbose
    draw_wing2(sections, vortex, cs,ra,rb, vortex_cs)
end

end

