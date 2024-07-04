process.stdout.write("{");
var max = 0xFFFF;

// for (let i = 0; i < 600; i++) {
//     process.stdout.write("" + Math.round(max / 2 + (max / 2 - 1) * Math.sin(2 * Math.PI * i / 600)) + ",");
// }
for (let i = 0; i < 100; i++) {
    process.stdout.write("" + Math.round(i * max / 100) + ",");
}

for (let i = 0; i < 200; i++) {
    process.stdout.write("" + max + ",");
}

for (let i = 99; i >= 0; i--) {
    process.stdout.write("" +  Math.round(i * max / 100) + ",");
}

for (let i = 0; i < 200; i++) {
    process.stdout.write("" + 0 + ",");
}

process.stdout.write("}\r\n");

