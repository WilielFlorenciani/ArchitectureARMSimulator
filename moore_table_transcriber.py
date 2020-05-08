import csv

micro_signals = open("micro_signals.txt", 'w')

with open('moore_table_sheet.txt') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter='\t')
    state = 345
    for row in csv_reader:
        micro_signals.write("%d'b" % (len(row)))
        for bit in row:
            micro_signals.write("%s" % (bit))
        micro_signals.write(", //%d\n" % (state))
        state += 1

micro_signals.close()