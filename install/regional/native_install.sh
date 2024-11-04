#!/bin/bash
set -e

VER=0.0.1
release="airy"
echo -e "\e[2;32mPerforming initial system update, please make sure you are connected to the internet.\e[0m"
sudo apt-get update
sudo apt-get dist-upgrade

LOGO=('\n\n                            \e[0m\e[38;5;252m              ▄▄▄▄▄▄▄▄'
'\e[2;34m         ▄▄▄▄▄ \e[2;33m▄▄▄▄▄\e[0m\e[38;5;252m                    ▀▀▀▀▀▀▀▀▀'
'\e[2;34m     ▄███████▀\e[2;33m▄██████▄\e[0m\e[38;5;252m   ▀█████████████████████▀'
'\e[2;34m  ▄██████████ \e[2;33m████████\e[31m ▄\e[0m\e[38;5;249m   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄'
'\e[2;34m ███████████▀ \e[2;33m███████▀\e[31m ██\e[0m\e[38;5;249m   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀'
'\e[2;34m█████████▀   \e[2;33m▀▀▀▀▀▀▀▀\e[31m ████\e[0m\e[38;5;246m   ▀███████████▀'
'\e[2;34m▀█████▀ \e[2;32m▄▄███████████▄\e[31m ████\e[0m\e[38;5;243m   ▄▄▄▄▄▄▄▄▄'
'\e[2;34m  ▀▀▀ \e[2;32m███████████████▀\e[31m ████\e[0m\e[38;5;243m   ▀▀▀▀▀▀▀▀'
'       \e[2;32m▀▀█████▀▀▀▀▀▀\e[31m  ▀▀▀▀\e[0m\e[38;5;240m   ▄█████▀'
'              \e[2;90m ████████▀    ▄▄▄'
'              \e[2;90m ▀███▀       ▀▀▀'
'              \e[2;90m  ▀▀      \e[0m'
'╔═══╗╔═══╗╔═══╗╔═╗ ╔╗╔══╗╔═══╗╔══╗╔╗   ╔═══╗╔════╗'
'║╔═╗║║╔═╗║║╔═╗║║║║ ║║╚╣╠╝║╔═╗║╚╣╠╝║║   ║╔═╗║║╔╗╔╗║'
'║║ ╚╝║║ ║║║║ ╚╝║║╚╗║║ ║║ ║║ ║║ ║║ ║║   ║║ ║║╚╝║║╚╝'
'║║   ║║ ║║║║╔═╗║╔╗╚╝║ ║║ ║╚═╝║ ║║ ║║   ║║ ║║  ║║  '
'║║ ╔╗║║ ║║║║╚╗║║║╚╗║║ ║║ ║╔══╝ ║║ ║║ ╔╗║║ ║║  ║║  '
'║╚═╝║║╚═╝║║╚═╝║║║ ║║║╔╣╠╗║║   ╔╣╠╗║╚═╝║║╚═╝║ ╔╝╚╗ '
'╚═══╝╚═══╝╚═══╝╚╝ ╚═╝╚══╝╚╝   ╚══╝╚═══╝╚═══╝ ╚══╝ '
'\e[38;5;252m\t  _____   _              \e[0m'
'\e[38;5;252m\t (_____) (_) _           \e[0m'
'\e[38;5;250m\t(_)___(_) _ (_)__  _   _ \e[0m'
'\e[38;5;248m\t(_______)(_)(____)(_) (_)\e[0m'
'\e[38;5;246m\t(_)   (_)(_)(_)   (_)_(_)\e[0m'
'\e[38;5;244m\t(_)   (_)(_)(_)    (____)\e[0m'
'\e[38;5;242m\t                    __(_)\e[0m'
'\e[38;5;240m\t                   (___) \e[0m\n'
)

for line in "${LOGO[@]}"; do
    echo -e "$line"
done

echo -e "\n\e[2;32mWelcome to the CogniPilot $release native installer ($VER) - Ctrl-c at any time to exit.\e[0m\n"

while :; do
  read -p $'\n\e[2;33mClone repositories with git using already setup github ssh keys [y/n]?\e[0m ' sshgit
  if [[ ${sshgit,,} == "y" ]]; then
    sshgit="y"
    echo -e "\e[2;32mUsing git with ssh, best for developers.\e[0m"
    break
  elif [[ ${sshgit,,} == "n" ]]; then
    sshgit="n"
    echo -e "\e[2;32mUsing git with https, read only.\e[0m"
    break
  else
    echo -e "\e[31mInvalid input please try again.\e[0m"
  fi
done

mkdir -p ~/cognipilot
cd ~/cognipilot

if [[ ${sshgit} == "y" ]]; then
  echo -e "\e[2;34mBUILD:\e[0m\e[2;32m Checking helmet version, updating if needed.\e[0m"
  if [ ! -f ~/cognipilot/helmet/.git/HEAD ]; then
    git clone -b $release git@github.com:CogniPilot/helmet.git
  elif ! grep -qF "$release" ~/cognipilot/helmet/.git/HEAD; then
    cd ~/cognipilot/helmet
    git checkout $release
    git pull
    cd ~/cognipilot
  else
    cd ~/cognipilot/helmet
    git pull
    cd ~/cognipilot
  fi
elif [[ ${sshgit} == "n" ]]; then
  echo -e "\e[2;34mBUILD:\e[0m\e[2;32m Checking read only helmet version, updating if needed.\e[0m"
  if [ ! -f ~/cognipilot/helmet/.git/HEAD ]; then
    git clone -b $release https://github.com/CogniPilot/helmet.git
  elif ! grep -qF "$release" ~/cognipilot/helmet/.git/HEAD; then
    cd ~/cognipilot/helmet
    git checkout $release
    git pull
    cd ~/cognipilot
  else
    cd ~/cognipilot/helmet
    git pull
    cd ~/cognipilot
  fi
fi

pushd ~/cognipilot/helmet/install

./scripts/base.sh

# install zeth config
sudo mkdir -p /opt/zeth

if ! [ -f  /opt/zeth/zeth.conf ]
then
  sudo cp ./resources/zeth.conf /opt/zeth
fi

if ! [ -f  /opt/zeth/net-setup.sh ]
then
  sudo cp ./resources/net-setup.sh /opt/zeth
fi

./scripts/zephyr.sh
./scripts/ros.sh
./scripts/gazebo.sh
./scripts/user_setup.sh native


# install scripts
cp ./resources/build_workspace ~/bin
cp ./resources/cyecca_poetry ~/bin
cp ./resources/docs ~/bin
cp ./resources/build_foxglove ~/bin

popd

echo -e "\e[2;32mCogniPilot native installer has finished!\nPlease source your .bashrc:\e[0m\e[31m\n\tsource ~/.bashrc\e[0m"
