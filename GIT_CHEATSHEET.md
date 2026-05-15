# Git-Cheatsheet — PumpLogger-Team

Für Peter & Zeno. Die ~10 Befehle die im Alltag wirklich vorkommen,
in der Reihenfolge wie sie auftreten.

---

## Setup (einmal nach `git clone`)

```sh
git remote add upstream https://github.com/zdavatz/fp-sns-stbox1.git
git config user.name  "Peter Schmidlin"
git config user.email "schmidlinp@gmail.com"
```

`origin` = dein Fork (`Peter8610/...`). `upstream` = Zeno's
Hauptrepo. Pushes gehen immer zu `origin`, gepullt wird von `upstream`.

---

## Neue Arbeit starten

```sh
git checkout main
git pull upstream main             # Zeno's neueste Version holen
git push origin main               # dein Fork auch updaten
git checkout -b fix/<thema>        # neuer Branch
```

Branch-Namen kurz & beschreibend: `fix/ble-read-timeout`,
`feat/gps-cn0-logging`, `docs/sensor-fusion-pdf`.

---

## Während der Arbeit

```sh
git status                         # was hat sich geändert?
git diff                           # WIE hat es sich geändert?
git add <datei1> <datei2>          # für Commit vormerken
git commit -m "kurze Nachricht"    # lokal sichern (NICHT auf GitHub)
```

Faustregel: kleine Commits, häufig. Pro logische Änderung ein Commit.
Du kannst 30× committen ohne dass Zeno irgendetwas davon mitkriegt.

---

## Zwischendurch zu GitHub hochladen

```sh
git push -u origin <branch-name>   # erstes Mal: mit -u
git push                            # alle weiteren Male
```

Pushen ist sicher: dein Branch ist eine private Spielwiese,
`main` wird dabei nicht angefasst.

---

## Pull Request öffnen (wenn fertig)

```sh
gh pr create --base main --title "kurze Beschreibung" \
  --body "Was geändert wurde + warum"
```

Oder im Browser auf github.com → "Compare & Pull Request"-Button.
Zeno wird benachrichtigt, kann kommentieren, du kannst Fixes
nachschieben (= commit + push → der PR aktualisiert sich
automatisch).

---

## Nach dem Merge — zurück zu main

```sh
git checkout main
git pull upstream main             # Zeno's mergede Version holen
git push origin main               # Fork auf Stand bringen
git branch -d <alter-branch>       # lokal löschen
git push origin --delete <alter-branch>   # auf GitHub löschen
```

Dann wieder bei "Neue Arbeit starten" — auf einem **frischen**
main, nicht auf dem alten.

---

## Sync mit Zeno (periodisch, z.B. wöchentlich)

```sh
git fetch upstream
git checkout main
git merge upstream/main
git push origin main
```

---

## Wenn was schiefgeht

Erster Schritt immer:

```sh
git status                         # wo stehe ich?
git log --oneline -10              # letzte 10 Commits
git diff HEAD                      # ungesicherte Änderungen
```

**Niemals** ohne Rückfrage:

| Befehl | Risiko |
|---|---|
| `git push --force` | überschreibt Remote, kann Zeno's Arbeit killen |
| `git reset --hard` | löscht ungesicherte Arbeit unwiederbringlich |
| `git clean -fd` | löscht alle nicht-getrackten Dateien |
| `git checkout .` | wirft alle ungesicherten Änderungen weg |

Bei Unsicherheit Claude fragen. Git macht 95% des Schadens
in den 5% der Fälle wo man "sicher schnell mal" eingibt.

---

## Mentaler Pfad

```
editieren → commit (lokal) → push (zu GitHub) →
PR öffnen → Zeno reviewt → Merge →
pull zurück auf main → neuer Branch → wiederholen
```

Eine Commit/Push-Iteration ist billig. Eine PR-Iteration kostet
Zeno's Aufmerksamkeit — daher PRs erst öffnen wenn die Arbeit
inhaltlich rund ist, nicht für jeden Zwischenstand.
