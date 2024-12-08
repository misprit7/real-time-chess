<h1 align="center">
  <!--<picture><img src="./doc/img/logo.png" height="400"/></picture>-->
  <br />
  Real Time Chess
</h1>
<h2 align="center">
  A physical chess board without the concept of turns
</h2>
<h2 align="center">
  Video explanation: [Coming soon!](https://youtu.be/y7VtSK23_Jg)
</h2>

# Pitch

Chess is boring. I'm boring too so I enjoy it anyways, but I can't help but think "I could design it better." Normally in chess players move sequentially in turns, but this introduces a huge latency bug that the developers of chess forgot to patch: you spend literally half the time waiting for your opponent!

The obvious solution is just get rid of the concept of turns in chess altogether and let players move whenever they want. Real time strategy games like StarCraft and Age of Empires are much more fun and spectator friendly than chess, so this should be a pretty uncontroversial minor rules update that can be implemented before the next world championship. To prevent things from getting too chaotic over the board each piece has an individual cooldown, so once it's been moved it can't move for a fixed period afterwards.

However there's an unfortunate roadblock to the widespread adoption of real time chess: as the [Niemann controversy](https://www.cnn.com/2023/09/26/sport/hans-niemann-denies-sex-toys-cheating-chess-spt-intl/index.html) has made all too clear, chess is [not immune](https://www.youtube.com/watch?v=QNuu8KTUEwU) from accusations of cheating through spectator **ass**istance or outside **anal**isys tools. Trying to have players self enforce these piece cooldowns is impossible. However where the intrinsic goodness of the human psyche fails, engineering is always ready to step in. This project is a physical chess board that keeps track and displays the cooldown remaining for each piece, and even physically holds them in place so no accidental cheating can occur.

# Design Files

The firmware and pcb kicad files are in this repo. For the physical design, see [the design on OnShape](https://cad.onshape.com/documents/ad9a4c8f8eed1cb460895551/w/e793072796f18b5ae284597f/e/b7ae406cfa566c5c0e6fed4b). Other than those parts that were cnced, here were the off the shelf components used:

- [Insulating washers](https://www.mcmaster.com/catalog/130/3683/95225A340): One under each electromagnet, to keep them isolated from the casing
- [Plastic screws](https://www.mcmaster.com/catalog/130/3446/92492A728): To attach electromagnets to base, plastic to prevent electrical connection
- [These](https://www.mcmaster.com/catalog/130/3435/92095A182) and [these](https://www.mcmaster.com/catalog/130/3435/92095A177) screws: to attach the internal supports and squares respectively
- [Spacers](https://www.mcmaster.com/catalog/94669A095): For an offset between the decorative and functional pcbs
- [Electromagnets](https://www.adafruit.com/product/3873): Most expensive part other than the machining, ~$600 per board. Could probably get them cheaper from China or something but for low quantities this was easiest

# Known Issues

- Power distribution: Traces on the pcbs are way undersized given there are many amps running through them, so there are large voltage drops when many pieces are on cooldown simultaneously. To solve this these traces should be much wider
- Tolerances: The pcbs have extremely tight tolerances which makes assmbling the board extremely annoying. The edges and holes should probably have more room
- Pin heights: The height of the pins for the banana connectors are taller than the mechanical design allows for, these are fairly easy to shorten using a dremel but probably something that should be fixed
- Corner screws: Given the order of assembly, it's impossible to insert/fasten the 4 corner screws
