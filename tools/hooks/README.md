# How to use

Copy the files in this directory to the .git/hooks/ directory
Linux
```
find tools/hooks -type f ! -name 'README.md' -exec cp {} .git/hooks/ \;
```
Windows
```
Get-ChildItem -Path tools/hooks -File -Exclude README.md | ForEach-Object {
    Copy-Item -Path $_.FullName -Destination .git/hooks
}
```
