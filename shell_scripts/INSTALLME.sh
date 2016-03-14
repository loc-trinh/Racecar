
echo ""
echo ""
function get_bashrc_contents()
{
ABSOLUTE_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cat << EOF


# ================================================
# This section defined shortcuts and paths for RSS-stuff
# Remove the section between the equals signs to uninstall

export PATH=\$PATH:$ABSOLUTE_PATH

alias sshcar="sshpass -p 'racecar@mit' ssh racecar@192.168.100.72"
alias cdws="cd $ABSOLUTE_PATH/../"
alias rosenv="env | grep ROS"
alias gs="git status"
gitpush() {
    CURRDIR="\$(pwd)"
    cdws
    gitpush.sh "\$1"
    cd \$CURRDIR
}
gitpull() {
    CURRDIR="\$(pwd)"
    cdws
    git pull
    cd \$CURRDIR
}
gitmakebranch() {
    CURRDIR="\$(pwd)"
    cdws
    git pull
    git checkout -b \$1
    git push -u origin \$1
    git branch --set-upstream-to=origin/\$1 \$1
    cd \$CURRDIR
}
cm(){
    CURRDIR="\$(pwd)"
    cdws
    cd ../..
    catkin_make -j
    cd \$CURRDIR
}
cmr(){
    CURRDIR="\$(pwd)"
    cdws
    cd ../..
    catkin_make -DCMAKE_BUILD_TYPE=Release
    cd \$CURRDIR
}
cmc(){
    CURRDIR="\$(pwd)"
    cdws
    cd ../..
    catkin_make clean
    catkin_make 
    cd \$CURRDIR
}
alias gpush=gitpush
alias gpull=gitpull
alias gmb=gitmakebranch
alias roslocal=". rosenv.sh 0"
alias rosremote=". rosenv.sh 1"

# ================================================
EOF
}

check_dep() {
	if [[ $(which $1) ]]; then
		echo "Dependency $1 already installed." 
	else
		echo "Dependency $1 required. installing..." 
		sudo apt-get install -y $1	
	fi
}

OUT=$(get_bashrc_contents)

if [ -a ~/.bashrc_bak ]; then
	echo "Restoring original ~/.bashrc..."
	cp ~/.bashrc_bak ~/.bashrc
else
	echo "Backing up ~/.bashrc -> ~/.bashrc_bak"
	cp ~/.bashrc ~/.bashrc_bak
fi

echo "Installing to ~/.bashrc" 
echo "Restore ~/.bashrc_bak to uninstall" 

echo "$OUT" >> ~/.bashrc
echo ""
echo "Done modifying ~/.bashrc. Changes take effect when terminal session is restarted." 
echo ""
echo "Now checking tools..." 
check_dep "sshpass"
check_dep "tmux"
