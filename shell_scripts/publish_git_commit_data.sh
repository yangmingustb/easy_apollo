
#使用git 统计
#统计所有人的提交
#git log --format='%aN' | sort -u | while read name; do echo -en "$name\t,"; git log --author="$name" --pretty=tformat: --numstat | awk '{ add += $1; subs += $2; loc += $1 - $2 } END { printf "新增行数: %s, 移除行数: %s, 总行数: %s\n", add, subs, loc }' -; done

#统计所有人的时间内的提交
# git log --format='%aN' | sort -u | while read name; do echo -en "$name:\t"; git log --author="$name" --pretty=tformat: --since="2019-08-01" --until="2023-07-22" --numstat | awk '{ add += $1; subs += $2; loc += $1 - $2 } END { printf "新增行数: %s, 移除行数: %s, 总行数: %s, commit数量: %s\n", add, subs, loc, wc -l}' -; done


#个人提交
#git log --author="userName" --pretty=tformat: --numstat | awk '{ add += $1; subs += $2; loc += $1 - $2 } END { printf "新增行数: %s, 移除行数: %s, 总行数: %s\n", add, subs, loc }'

#个人时间段内的提交
#git log --author="userName" --pretty=tformat: --since="2019-08-01" --until="2019-10-01" --numstat | awk '{ add += $1; subs += $2; loc += $1 - $2 } END { printf "新增行数: %s, 移除行数: %s, 总行数: %s\n", add, subs, loc }'

############################################
#使用git stats统计
#sudo apt install gnuplot


#git clone --depth 1 git@github.com:hoxu/gitstats.git

#cd gitstats/

#git fetch origin pull/105/head:pr105

#git checkout pr105 

python gitstats ./../jimu-auto ./jimu
