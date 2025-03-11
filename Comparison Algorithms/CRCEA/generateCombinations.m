function newSeqList = generateCombinations(A, B, Seq)
    % 找到 Seq 中需要替换的元素的位置
    replaceIndices = ismember(Seq, A);
    
    % 获取需要替换的元素的索引
    replacePos = find(replaceIndices);
    
    % 计算所有替换组合的数量
    numCombinations = length(B) ^ length(A);
    
    % 初始化新的 Seq 集合
    newSeqList = {};
    
    % 对 B 中的每个元素的组合进行遍历
    for i = 0:numCombinations-1
        % 创建新的 Seq 拷贝
        tempSeq = Seq;
        
        % 获取当前组合的 B 中的元素索引
        combination = i;
        
        % 对 A 中的每个元素进行统一替换
        for j = 1:length(A)
            % 计算当前需要替换成 B 中哪个元素
            replaceValue = B(mod(combination, length(B)) + 1);
            % 将 A 中的元素替换为 B 中的当前元素
            tempSeq(replacePos(Seq(replacePos) == A(j))) = replaceValue;
            
            % 更新组合索引
            combination = floor(combination / length(B));
        end
        
        % 将替换后的 Seq 添加到集合中
        newSeqList{end+1} = tempSeq;
    end
end
