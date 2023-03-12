Tasks are distributed as issues, each person is expected to work on a particular issue, or more than one person can work on the same issue if this issue needs more than one sub-team.

When working on a particular issue, first you need to make a branch in the following format
`branch_name/issue_number`
- Where  `branch_name` is one of `software, embedded, data and devops`.
- And `issue_number` is the number of issues listed in github repo, number from `1 to 25`
An example of the branch name is `software/3`, `embedded/12`.


To view branches
```bash 
❯ git branch
  data
  devops
  embedded
  main
* software
```

Before creating a branch, first go to the branch you want to create a new branch based on, I will go to `software` branch.

To create a branch (will assume working in a software issue, change software and issue number accordingly)
```bash
git checkout software
```

Then create the branch with the issue number as described above.
```bash
git checkout -b software/3
```

now you are in the `software/3` branch who is associated with the issue-3 in the software branch, and yo can start making changes and adding commits.

After finishing your work on that issue branch, you need to merge it with the base branch (in this case `software`)
