set dotenv-load

[private]
default:
    @just --list --unsorted

# Copy repo content to remote host with 'rsync' and watch for changes
sync hostname="${ROBOT_NAMESPACE}" password="husarion": _install-rsync _run-as-user
    #!/bin/bash
    mkdir -m 775 -p maps
    sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --delete ./ husarion@{{hostname}}:/home/husarion/avdr_ws/src/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ --exclude='.git/' --exclude='maps/' ; do
        sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --delete ./ husarion@{{hostname}}:/home/husarion/avdr_ws/src/${PWD##*/}
    done

_run-as-user:
    #!/bin/bash
    if [ "$EUID" -eq 0 ]; then
        echo -e "\e[1;33mPlease re-run as non-root user\e[0m"
        exit 1
    fi

_install-rsync:
    #!/bin/bash
    if ! command -v rsync &> /dev/null || ! command -v sshpass &> /dev/null || ! command -v inotifywait &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit 1
        fi
        apt install -y rsync sshpass inotify-tools
    fi
