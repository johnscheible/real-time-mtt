function [annos]=read_annotation(filename)
fp = fopen(filename, 'r');
if(fp < 0) 
	disp(['file not exist : ' filename]);
end

annos = struct('fname', {}, 'rts', {}, 'binfo', {});
anno_idx = 1;

last_fid = -1;

while(1)
	tline = fgets(fp);
	if ~ischar(tline)
		break;
	end

	idx = 1;
	iname = sscanf(tline, '\"%s\"');
    
    fid = sscanf(iname, 'left/image_%08d_0.png');
    if(last_fid > 0)
        gap = fid - last_fid - 1;
        assert(gap >= 0);
        
        for j = 1:gap
            annos(anno_idx).binfo = false;
            anno_idx = anno_idx + 1;
        end
    end
    last_fid = fid;
    
	if(iname(end) == ':')
		% disp(iname(1:end-2));
		idx = idx + length(iname) + 1;
		annos(anno_idx).fname = iname;
		while(1)
			[rt, next] = getRect(tline(idx:end));
			if(next == 0)
				break;
			end
			annos(anno_idx).rts(end+1, :) = rt(:)';
            
			idx = idx + next;
		end
		annos(anno_idx).binfo = true;
	else
		annos(anno_idx).binfo = false;
	end
	anno_idx = anno_idx + 1;
end

fclose(fp);
end


function [rtout, idx] = getRect(string)

idx = 1;
while(string(idx) ~= '(' && idx < length(string))
	idx = idx + 1;
end

if(length(string) > 0 && string(idx) == '(')
	[rt, d1, d2, next] = sscanf(string(idx+1:end), '%d, %d, %d, %d');
	idx = idx + next;
    
    rtout(1) = min(rt(1), rt(3));
    rtout(2) = min(rt(2), rt(4));
    rtout(3) = max(rt(1), rt(3));
    rtout(4) = max(rt(2), rt(4));
    
    assert(rtout(3) > rtout(1));
    assert(rtout(4) > rtout(2));
%     rtout = rt;
else
	rtout = [];
	idx = 0;
end

end
