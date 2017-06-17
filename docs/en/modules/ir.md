# IR Module
| Since  | Origin / Contributor  | Maintainer  | Source  |
| :----- | :-------------------- | :---------- | :------ |
| 2017-02-22 | [Giorgio Massussi](https://github.com/brainz73) | [Giorgio Massussi](https://github.com/brainz73) | [ir.c](../../../app/modules/ir.c)|


This module enables to send and receive infra-red signals using IR leds and IR receivers; it is inspired by [IRremote Arduino Library](http://www.righto.com/2009/08/multi-protocol-infrared-remote-library.html).
An IR signal is a sequence of successives mark and space; a mark is the time when the IR led is ON (modulated at 38Khz), a space is the time when the IR led is OFF.
The module provides functions for read/write raw signals as arrays of successive mark and space lengths, and a function to encode a binary signal with extended [NEC protocol](http://www.sbprojects.com/knowledge/ir/nec.php).


## ir.config()

Initialize parameters for binary signal encoding with extended [NEC protocol](http://www.sbprojects.com/knowledge/ir/nec.php).
A signal is composed by one or more frames of bits. Each bit is encoded as a mark of fixed length followed by a space: the space length determines the bit value.
Each frame starts with a single "mark and space" header and ends with a single mark; a space (pause) is inserted between frames.

#### Syntax
`ir.config(header, headerspace, mark, space0, space1 [, pause])`

#### Parameters
- `header` time in microseconds of the header mark
- `headerspace` time in microseconds of the header space
- `mark` time in microseconds of a mark
- `space0` time in microseconds of the space for a 0 bit
- `space1` time in microseconds of the space for a 1 bit
- `pause` optional space between multiple frames

#### Returns
`nil`

#### Example
```lua
ir.config(3500,1750,435,435,1300,9000)
```
#### See also
- [`ir.writebits()`](#irwritebits)

## ir.read()

Read a raw signal from a ir receiver; this function is asynchrounous: the function returns immediately, when a signal is detected or the global timeout is reached a callback is invoked with the received data.

#### Syntax
`ir.read(pin, timeout, pulse_timeout, callback)`

#### Parameters
`pin` pin to read, IO index
`timeout` global timeout in microseconds
`pulse_timeout` timeout for a single mark/space pulse
`callback` callback function invoked when signal ends or when global timeout occurs. An array of successive mark and space length in microseconds is passed to the callback.
The first value is the first mark length, the last value is the last mark length.


#### Returns
`1`

#### Example
```lua
-- read value of gpio 0.
ir.read(0)
```

## ir.write()

Write a raw signal; this function is asynchrounous: the function returns immediately, when a signal is sent a callback, if present, is invoked.

#### Syntax
`ir.write(pin, data [, callback]])`

#### Parameters
- `pin`  pin to use, IO index
- `data` An array of successive mark and space length in microseconds; the first value is the first mark length, the last value is the last mark length.
- `callback` an optional callback function, if present is invoked when signal is sent.


#### Returns
`nil`

#### Example
```lua
ir.write(1,{3500,1750,435,435,435,1300,435}) -- Send a signal 
```

## ir.writebits()

Write a binary signal encoded with extended [NEC protocol](http://www.sbprojects.com/knowledge/ir/nec.php).
A signal is composed by one or more frames of bits. Each bit is encoded as a mark of fixed length followed by a space: the space length determines the bit value.
Each frame starts with a single "mark and space" header and ends with a single mark; a space (pause) is inserted between frames.
This function is asynchrounous: the function returns immediately, when the signal is sent a callback, if present, is invoked.

#### Syntax
`ir.writebits(pin, frame_length, data [, frame_length, data [,...] ] [, callback])`

#### Parameters
- `pin`  pin to use, IO index
- `frame-length`  Frame length in bits
- `data`  An array of bytes representing the frame; array length must be sufficient for the corresponding frame_length, extra bits or bytes are ignored;
- `callback` an optional callback function, if present is invoked when signal is sent.

#### Returns
`nil`

#### Example

```lua
ir.writebits(1, 6, { 0xF0 }, 12, { 0x0F, 0xFF }) -- sends a signal composed by two binary frames: b000011 and b000011111111
```

#### See also
[`ir.config()`](#irconfig)


