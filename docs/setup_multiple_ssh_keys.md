# Setting Up Multiple SSH Keys 
For different github accounts using one device (RPi)

## Generate SSH Keys
```shell
ssh-keygen -t ed25519 -C "<your_email@example.com>"
```
**Important: ** when prompt 
```shell
Enter a file in which to save the key (/home/ubuntu/.ssh/id_ed25519): 
```
**DO NOT** press `Enter`, instead, rename your ssh key by typing:
```shell
/home/ubuntu/.ssh/id_ed25519_<username>
```
For next persons, repeat the above and make sure the name of your ssh key is different from others.

## Adding your SSH key to the ssh-agent
1. Start the ssh-agent in the background.
```shell
eval "$(ssh-agent -s)"
```
2. Add each collaborator's SSH key to the ssh-agent.
```shell
ssh-add ~/.ssh/id_ed25519_<user1>
ssh-add ~/.ssh/id_ed25519_<user2>
ssh-add ~/.ssh/id_ed25519_<user3>
```

## Modify the SSH Config
Open a text editor to create or modify SSH config file:
```shell
nano ~/.ssh/config  
```
> if you know how to use `vim` go ahead with the command: `vim ~/.ssh/config`

Edit the file so that it looks like:
```yaml
#user1 account
Host github.com-<user1's github account>
	HostName github.com
	User git
	IdentityFile ~/.ssh/id_ed25519_<user1>

#user3 account
Host github.com-<user2's github account>
	HostName github.com
	User git
	IdentityFile ~/.ssh/id_ed25519_<user2>

#user3 account
Host github.com-<user3's github account>
	HostName github.com
	User git
	IdentityFile ~/.ssh/id_ed25519_<user3>
```

Currently, our SSH config should be like:
```yaml
#ole account
Host github.com-kjorholt
	HostName github.com
	User git
	IdentityFile ~/.ssh/id_ed25519

#lin account
Host github.com-linzhanguca
	HostName github.com
	User git
	IdentityFile ~/.ssh/id_ed25519_zhang

#corbin account
Host github.com-kingkorb
	HostName github.com
	User git
	IdentityFile ~/.ssh/id_ed25519_humphrey
```

## Add SSH Key to Github account
Simply follow [this guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
