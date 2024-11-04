#!/bin/bash
set -e

VER=0.0.1
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
'\e[5m\e[32m _   _         _                                _ '
'| | | |       (_)                              | |'
'| | | | _ __   _ __   __ ___  _ __  ___   __ _ | |'
'| | | || `_ \ | |\ \ / // _ \| `__|/ __| / _` || |'
'| |_| || | | || | \ V /|  __/| |   \__ \| (_| || |'
' \___/ |_| |_||_|  \_/  \___||_|   |___/ \__,_||_|'
' _____             _          _  _                '
'|_   _|           | |        | || |               '
'  | |  _ __   ___ | |_  __ _ | || |  ___  _ __    '
'  | | | `_ \ / __|| __|/ _` || || | / _ \| `__|   '
' _| |_| | | |\__ \| |_| (_| || || ||  __/| |      '
' \___/|_| |_||___/ \__|\__,_||_||_| \___||_|      \e[0m')

for line in "${LOGO[@]}"; do
    echo -e "$line"
done

echo -e "\n\e[2;32mWelcome to the CogniPilot universal installer ($VER) - Ctrl-c at any time to exit.\e[0m\n"

PS3=$'\n\e[2;33mEnter a CogniPilot release (number) to use: \e[0m'
select opt in airy brave main; do
  case $opt in
  airy)
    release=airy
    echo -e "\e[2;32mUsing CogniPilot release airy alicanto.\n\e[0m"
    break;;
  brave)
    release=brave
    echo -e "\e[2;32mUsing CogniPilot release brave bennu.\n\e[0m"
    break;;
  main)
    release=main
    echo -e "\e[2;32mUsing CogniPilot main development branch.\n\e[0m"
    break;;
  *)
    echo -e "\e[31mInvalid option $REPLY\n\e[0m";;
  esac
done

PS3=$'\n\e[2;33mEnter a CogniPilot installer type (number) to use: \e[0m'
select opt in native navqplus; do
  case $opt in
  native)
    installer=native_install.sh
    echo -e "\e[2;32mUsing CogniPilot installer for native.\n\e[0m"
    break;;
  navqplus)
    installer=navqplus_install.sh
    echo -e "\e[2;32mUsing CogniPilot installer for navqplus.\n\e[0m"
    break;;
  *)
    echo -e "\e[31mInvalid option $REPLY\n\e[0m";;
  esac
done

mkdir -p ~/cognipilot/installer

wget -O ~/cognipilot/installer/$installer https://raw.githubusercontent.com/CogniPilot/helmet/$release/install/$installer
chmod a+x ~/cognipilot/installer/$installer
/bin/bash ~/cognipilot/installer/$installer
