function [out] = parse_JVL_file(filename)
%PARSE_JVL_FILE Reads critical parameters from JVL File

fid = fopen(filename);
out = textscan(fid, '%s\n');
%fclose(fid)
% 
% l = fgetl(fid);
% i = 1;
% while 1
% %while i <= 100
%    %disp([num2str(i),':',l]) ;
%    s = split(l);
%    disp([num2str(i),':',s{1,1}]);
%    if strcmp(s{1,1}, 'SURFACE')
%        disp('New Surface')
%    end
%    l = fgetl(fid);
%    i = i+1;
%    if ~ischar(l); break;  %end of file
% end
% out.data = 1;
% end
% 
