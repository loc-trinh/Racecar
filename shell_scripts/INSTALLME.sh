echo "Installing to ~/.bashrc" 
echo "Please do not run more than once." 
echo "Delete contents from end of ~/.bashrc to uninstall" 

function get_bashrc_contents()
{
ABSOLUTE_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cat << EOF


# ================================================
# This section defined shortcuts and paths for RSS-stuff
# Remove the section between the equals signs to uninstall

export PATH=\$PATH:$ABSOLUTE_PATH

alias sshcar="ssh -X racecar@192.168.100.63"
alias cdws="cd $ABSOLUTE_PATH/../"
alias rosenv="env | grep ROS"
alias gs="git status"
gitpush() {
    CURRDIR="$(pwd)"
    cdws
    gitpush.sh
    cd CURRDIR
}
gitpull() {
    CURRDIR="$(pwd)"
    cdws
    git pull
    cd CURRDIR
}
alias gpush=gitpush
alias gpull=gitpull
alias roslocal=". rosenv.sh 0"
alias rosremote=". rosenv.sh 1"

# ================================================
EOF
}

OUT=$(get_bashrc_contents)
echo "$OUT" >> ~/.bashrc
echo ""
echo "Done installing. Changes take effect when terminal session is restarted." 
