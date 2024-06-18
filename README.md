<h1 align="center">
  <!--<picture><img src="./doc/img/logo.png" height="400"/></picture>-->
  <br />
  Real Time Chess
</h1>
<h2 align="center">
  A physical chess board without the concept of turns
</h2>
<h2 align="center">
  Video explanation: Coming soon!
</h2>

# Pitch

Chess is boring. I'm boring too so I enjoy it anyways, but I can't help but think "I could design it better." Normally in chess players move sequentially in turns, but this introduces a huge latency bug that the developers of chess forgot to patch: you spend literally half the time waiting for your opponent!

The obvious solution is just get rid of the concept of turns in chess altogether and let players move whenever they want. Real time strategy games like StarCraft and Age of Empires are much more fun and spectator friendly than chess, so this should be a pretty uncontroversial minor rules update that can be implemented before the next world championship. To prevent things from getting too chaotic over the board each piece has an individual cooldown, so once it's been moved it can't move for a fixed period afterwards.

However there's an unfortunate roadblock to the widespread adoption of real time chess: as the [Niemann controversy](https://www.cnn.com/2023/09/26/sport/hans-niemann-denies-sex-toys-cheating-chess-spt-intl/index.html) has made all too clear, chess is [not immune](https://www.youtube.com/watch?v=QNuu8KTUEwU) from accusations of cheating through spectator **ass**istance or outside **anal**isys tools. Trying to have players self enforce these piece cooldowns is impossible. However where the intrinsic goodness of the human psyche fails, engineering is always ready to step in. This project is a physical chess board that keeps track and displays the cooldown remaining for each piece, and even physically holds them in place so no accidental cheating can occur.

