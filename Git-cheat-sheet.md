
# Getting the repo on your machine 
First clone the repo on your machine, copy the `url` of the repo and open a terminal in the folder you want to download the repo in.
```bash
❯ git clone https://github.com/eslamdyab21/AMRs-in-Warehouse-Systems.git
Cloning into 'AMRs-in-Warehouse-Systems'...
remote: Enumerating objects: 186, done.
remote: Counting objects: 100% (186/186), done.
remote: Compressing objects: 100% (130/130), done.
remote: Total 186 (delta 97), reused 128 (delta 51), pack-reused 0
Receiving objects: 100% (186/186), 643.28 KiB | 393.00 KiB/s, done.
Resolving deltas: 100% (97/97), done.

❯ ls
AMRs-in-Warehouse-Systems

❯ cd AMRs-in-Warehouse-Systems/
❯ ls
'Graduation Project Documentation'   LICENSE   map2d_update_robots_shelves_locations.py   README-images   README.m
```
Now you're in the `AMRs-in-Warehouse-Systems` repo. but if you `git branch` to view the branches you will find only the `main` branch.
</br>

```bash
❯ git branch
* main
```

</br>
To get other branches, use `git branch -a`, you will get other branches.

```bash
❯ git branch -a
* main
  remotes/origin/HEAD -> origin/main
  remotes/origin/data
  remotes/origin/devops
  remotes/origin/embedded
  remotes/origin/feature/software/3
  remotes/origin/main
  remotes/origin/software
```

but they are in the `remote (github website)` not in your machine, you can jump in one of them with `git checkout`

```bash
❯ git checkout remotes/origin/feature/software/3
Note: switching to 'remotes/origin/feature/software/3'.

You are in 'detached HEAD' state. You can look around, make experimental
changes and commit them, and you can discard any commits you make in this
state without impacting any branches by switching back to a branch.

If you want to create a new branch to retain commits you create, you may
do so (now or later) by using -c with the switch command. Example:

  git switch -c <new-branch-name>

Or undo this operation with:

  git switch -

Turn off this advice by setting config variable advice.detachedHead to false

HEAD is now at 32bc437 updating git flow docs
```

</br>
As described above, any changes you make will be lost unless you make a copy of that remote branch in your machine by `git switch -c <new-branch-name>`, you will make the `<new-branch-name>` as the branch name you jumped into without the `remotes/origin/` at the beginning, meaning the name will be in this case `feature/software/3`

```bash
❯ git switch -c "feature/software/3"
Switched to a new branch 'feature/software/3'

❯ ls
'Graduation Project Documentation'   LICENSE   map2d_update_robots_shelves_locations.py   README-images   README.md   warehouse-robots-shelves-simulation

❯ git status
On branch feature/software/3
nothing to commit, working tree clean

❯ git branch
* feature/software/3
  main
```

</br>
Now you have local copy of the branch in your machine and you can start making any changes you want and committing and pushing.



</br>
</br>
</br>
</br>

# Git flow
Tasks are distributed as issues, each person is expected to work on a particular issue, or more 
than one person can work on the same issue if this issue needs more than one sub-team.

</br>

When working on a particular issue, first you need to make a branch in the following format
`feature/branch_name/issue_number`
- Where  `branch_name` is one of `software, embedded, data and devops`.
- And `issue_number` is the number of issues listed in github repo, number from `1 to 25`
An example of the branch name is `feature/software/3`, `feature/embedded/12`.

</br>

</br>
To view branches

```bash 
❯ git branch
  data
  devops
  embedded
  main
* software
```

</br>
Before creating a branch, first go to the branch you want to create a new branch based on, I will go to `software` branch.
</br>
To create a branch (will assume working in a software issue, change software and issue number accordingly)

```bash
git checkout software
```

</br>
Then create the branch with the issue number as described above.

```bash
❯ git checkout -b feature/software/3
Switched to a new branch 'feature/software/3'
 
❯ ls
'Graduation Project Documentation'   LICENSE   map2d_update_robots_shelves_locations.py   README-images   README.md   warehouse-robots-shelves-simulation
  
❯ git status
On branch feature/software/3
nothing to commit, working tree clean
```

</br>

now you are in the `feature/software/3` branch who is associated with the issue-3 in the software branch, and yo can start making changes and adding commits.

</br>
</br>

Remeber to inclide `- fixes #issue_number` in your commit so that the change you made is connected to that issue.
ex: 

```bash
git commit -m 'create a function xyz - fixes #3'
```

</br>
Note that first push after making some commits will create the branch on github website

```bash
git push --set-upstream origin feature/software/3
```
After this first push, any other pushes will be normal with 

```bash
git push
```

</br>
</br>

After finishing your work on that issue branch, you need to make a pull request to merge it with the base branch (in this case `software`)


