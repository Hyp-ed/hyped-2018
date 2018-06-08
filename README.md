[![Build](https://travis-ci.org/Hyp-ed/hyped-2018.svg?branch=develop)]()

<p align="center">
  <img src="./logo.png"  />
</p>

This is the official repository of HYPED. HYPED is a student society at the University of Edinburgh dedicated to accelerating the development of Hyperloop and implementing the technology in the UK. HYPED is advancing both technical and commercial development of Hyperloop, having seen success in two international competitions. https://hyp-ed.com/

# Contributing

The following is a set of guidelines for contributing to HYPED. These are mostly guidelines, not rules. Use your best judgment, and feel free to propose changes to this document in a pull request.

## Coding Style

Since we will have dozens of developers coding for the pod, it is very important that everyone adheres to the same code style. For that purpose we have created the following [style guide](https://hyp-ed.github.io/styleguide/).

## Git

Git is a prerequisite for this project. If you have not attended any git workshop yet or are not permitted to code, please ask Brano for further instructions.

### Workflow

For our project we are using the [Gitflow](http://nvie.com/posts/a-successful-git-branching-model/) workflow. Instead of a single master branch, this workflow uses two branches to record the history of the project. The master branch stores the official release history, and the develop branch serves as an integration branch for features. Each new feature should reside in its own branch, which can be pushed to the central repository for backup/collaboration. But, instead of branching off of master, feature branches use develop as their parent branch. When a feature is complete, it gets merged back into develop. Features should never interact directly with master.

<p align="center">
  <img width="600"  src="./gitflow.png" />
</p>

### Rules

* Always use `git status`.

* Perform work in a feature branch.
  _Why:_
  > Because this way all work is done in isolation on a dedicated branch rather than the main branch. It allows you to submit multiple pull requests without confusion. You can iterate without polluting the master branch with potentially unstable, unfinished code. [Read more...](https://www.atlassian.com/git/tutorials/comparing-workflows#feature-branch-workflow)
* Always use `git pull --rebase` while working in a feature branch.

  _Why:_

  > Because `git pull` (without `--rebase`) would create merge commits which only clutter up the history without providing any useful information.

* Branch out from `develop`.

  _Why:_

  > This way, you can make sure that code in master will almost always build without problems.

* Never push into `develop` or `master` branch. Make a pull request.

  _Why:_

  > It notifies team members that they have completed a feature. It also enables and enforces easy code reviews.

* Delete local and remote feature branches after merging.

  _Why:_

  > It will clutter up your list of branches with dead branches. It insures you only ever merge the branch back into (`master` or `develop`) once. Feature branches should only exist while the work is still in progress.

* Before making a pull request, make sure your feature branch builds successfully and passes all tests (including code style checks).

* Write good commit messages. Write in imperative mood style and capitalise the messages.

  > You are welcome to use `git commit` without the `-m "<message>"` flag. This will bring up a text editor (likely nano or vim so make sure you know how to use those first). You can write your short commit message on the first line and then a longer description on the next line. The longer description should explain _why_ you did what you did and should be phrased as if it was an email. Don't use this for small and obvious commits.

### Do **NOT**

* Force push (`git push -f`)
* Squash branches

### Everyday workflow

* Checkout a feature branch.
  ```sh
  git checkout <subteam-feature>
  ```
* Make changes.

  ```sh
  git add
  git commit -m "commit"
  ```

* Sync with remote to get changes you have missed.

  ```sh
  git pull --rebase
  ```

* If you have conflicts [resolve them](https://help.github.com/articles/resolving-a-merge-conflict-using-the-command-line/) and continue rebase. (Remember **not** to commit.)
  ```sh
  git add <file1> <file2> ...
  git rebase --continue
  ```
* Push your branch.
  ```sh
  git push origin <subteam-feature>
  ```

### Start a new feature

* Start a new feature branch. The prefix of the branch should be the standard abbreviation of your subteam.

  ```sh
  git checkout develop
  git checkout -b <subteam-feature>
  git push -u origin <subteam-feature>
  ```

  #### Standard abbreviations

  `mgt` - Management (Heads)  
   `stm` - State Machine  
   `nav` - Navigation  
   `cmn` - Communications  
   `lib` - Libraries  
   `mot` - Motor Control  


### Submit completed feature

* Make a pull request and resolve conflicts.
* Pull requests will be accepted, merged and closed by a reviewer.
* Remove your local feature branch if you are done.

  ```sh
  git branch -d <subteam-feature>
  ```
