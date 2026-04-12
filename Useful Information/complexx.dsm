E000: 0F       sei  
E001: 8E 00 7F lds  #$007F
E004: CE 40 00 ldx  #$4000
E007: 6F 01    clr  $01,x
E009: 6F 03    clr  $03,x
E00B: 86 FF    lda  #$FF
E00D: 6F 00    clr  $00,x
E00F: A7 02    sta  $02,x
E011: 86 05    lda  #$05
E013: A7 01    sta  $01,x
E015: 4A       deca 
E016: A7 03    sta  $03,x
E018: 86 01    lda  #$01
E01A: 97 17    sta  $17
E01C: 0E       cli  
E01D: 20 FE    bra  $E01D
E01F: 8E 00 7F lds  #$007F
E022: B6 40 00 lda  $4000
E025: CE E7 B4 ldx  #$E7B4
E028: DF 53    stx  $53
E02A: CE 00 30 ldx  #$0030
E02D: DF 4F    stx  $4F
E02F: 0E       cli  
E030: 84 1F    anda #$1F
E032: 27 0F    beq  $E043
E034: 81 1F    cmpa #$1F
E036: 22 0B    bhi  $E043
E038: 4A       deca 
E039: 48       asla 
E03A: CE F2 8B ldx  #$F28B
E03D: 8D 06    bsr  $E045
E03F: EE 00    ldx  $00,x
E041: AD 00    jsr  $00,x
E043: 20 FE    bra  $E043
E045: DF 51    stx  $51
E047: 9B 52    adda $52
E049: 97 52    sta  $52
E04B: 96 51    lda  $51
E04D: 89 00    adca #$00
E04F: 97 51    sta  $51
E051: DE 51    ldx  $51
E053: 39       rts  
E054: DE 1E    ldx  $1E
E056: E6 00    ldb  $00,x
E058: 08       inx  
E059: DF 1E    stx  $1E
E05B: DE 20    ldx  $20
E05D: E7 00    stb  $00,x
E05F: 08       inx  
E060: DF 20    stx  $20
E062: 4A       deca 
E063: 26 EF    bne  $E054
E065: 39       rts  
E066: CE E2 AD ldx  #$E2AD
E069: DF 1E    stx  $1E
E06B: CE 00 30 ldx  #$0030
E06E: DF 20    stx  $20
E070: 86 10    lda  #$10
E072: BD E0 54 jsr  $E054
E075: CE 02 80 ldx  #$0280
E078: DF 22    stx  $22
E07A: DF 24    stx  $24
E07C: 86 08    lda  #$08
E07E: 97 1C    sta  $1C
E080: CE 00 30 ldx  #$0030
E083: DF 1E    stx  $1E
E085: DE 1E    ldx  $1E
E087: A6 00    lda  $00,x
E089: B7 40 02 sta  $4002
E08C: 08       inx  
E08D: 8C 00 40 cmpx #$0040
E090: 26 03    bne  $E095
E092: CE 00 30 ldx  #$0030
E095: DF 1E    stx  $1E
E097: CE 00 10 ldx  #$0010
E09A: 09       dex  
E09B: 26 FD    bne  $E09A
E09D: DE 22    ldx  $22
E09F: 09       dex  
E0A0: DF 22    stx  $22
E0A2: 26 E1    bne  $E085
E0A4: DE 24    ldx  $24
E0A6: DF 22    stx  $22
E0A8: 7A 00 1C dec  $001C
E0AB: 27 12    beq  $E0BF
E0AD: CE 00 30 ldx  #$0030
E0B0: A6 00    lda  $00,x
E0B2: 16       tab  
E0B3: 59       rolb 
E0B4: 49       rola 
E0B5: A7 00    sta  $00,x
E0B7: 08       inx  
E0B8: 8C 00 40 cmpx #$0040
E0BB: 26 F3    bne  $E0B0
E0BD: 20 C6    bra  $E085
E0BF: 39       rts  
E0C0: CE 00 30 ldx  #$0030
E0C3: 86 10    lda  #$10
E0C5: BD E1 55 jsr  $E155
E0C8: CE 06 00 ldx  #$0600
E0CB: DF 22    stx  $22
E0CD: DF 24    stx  $24
E0CF: C6 08    ldb  #$08
E0D1: D7 1D    stb  $1D
E0D3: 86 08    lda  #$08
E0D5: 97 18    sta  $18
E0D7: 97 1A    sta  $1A
E0D9: 86 0C    lda  #$0C
E0DB: 97 19    sta  $19
E0DD: 97 1B    sta  $1B
E0DF: CE E1 DA ldx  #$E1DA
E0E2: DF 1E    stx  $1E
E0E4: CE 00 30 ldx  #$0030
E0E7: DF 20    stx  $20
E0E9: 86 10    lda  #$10
E0EB: BD E1 3B jsr  $E13B
E0EE: CE 00 30 ldx  #$0030
E0F1: DF 1E    stx  $1E
E0F3: DF 20    stx  $20
E0F5: DE 1E    ldx  $1E
E0F7: E6 00    ldb  $00,x
E0F9: 7A 00 1A dec  $001A
E0FC: 26 0F    bne  $E10D
E0FE: 96 18    lda  $18
E100: 97 1A    sta  $1A
E102: 08       inx  
E103: 8C 00 40 cmpx #$0040
E106: 26 03    bne  $E10B
E108: CE 00 30 ldx  #$0030
E10B: DF 1E    stx  $1E
E10D: DE 20    ldx  $20
E10F: EB 00    addb $00,x
E111: CB 80    addb #$80
E113: F7 40 02 stb  $4002
E116: 7A 00 1B dec  $001B
E119: 26 0F    bne  $E12A
E11B: 96 19    lda  $19
E11D: 97 1B    sta  $1B
E11F: 08       inx  
E120: 8C 00 40 cmpx #$0040
E123: 26 03    bne  $E128
E125: CE 00 30 ldx  #$0030
E128: DF 20    stx  $20
E12A: DE 22    ldx  $22
E12C: 09       dex  
E12D: DF 22    stx  $22
E12F: 26 C4    bne  $E0F5
E131: DE 24    ldx  $24
E133: DF 22    stx  $22
E135: 7A 00 1D dec  $001D
E138: 26 A5    bne  $E0DF
E13A: 39       rts  
E13B: 97 1C    sta  $1C
E13D: DE 1E    ldx  $1E
E13F: E6 00    ldb  $00,x
E141: 08       inx  
E142: DF 1E    stx  $1E
E144: DE 20    ldx  $20
E146: A6 00    lda  $00,x
E148: 59       rolb 
E149: 49       rola 
E14A: A7 00    sta  $00,x
E14C: 08       inx  
E14D: DF 20    stx  $20
E14F: 7A 00 1C dec  $001C
E152: 26 E9    bne  $E13D
E154: 39       rts  
E155: 6F 00    clr  $00,x
E157: 08       inx  
E158: 4A       deca 
E159: 26 FA    bne  $E155
E15B: 39       rts  
E15C: CE E1 DA ldx  #$E1DA
E15F: DF 1E    stx  $1E
E161: CE 00 30 ldx  #$0030
E164: DF 20    stx  $20
E166: 86 10    lda  #$10
E168: BD E0 54 jsr  $E054
E16B: CE 08 00 ldx  #$0800
E16E: DF 22    stx  $22
E170: DF 1C    stx  $1C
E172: CE 00 30 ldx  #$0030
E175: DF 1E    stx  $1E
E177: DF 20    stx  $20
E179: 86 08    lda  #$08
E17B: 97 18    sta  $18
E17D: 97 1A    sta  $1A
E17F: 86 10    lda  #$10
E181: 97 19    sta  $19
E183: 97 1B    sta  $1B
E185: DE 1E    ldx  $1E
E187: E6 00    ldb  $00,x
E189: 7A 00 1A dec  $001A
E18C: 26 0F    bne  $E19D
E18E: 96 18    lda  $18
E190: 97 1A    sta  $1A
E192: 08       inx  
E193: 8C 00 40 cmpx #$0040
E196: 26 03    bne  $E19B
E198: CE 00 30 ldx  #$0030
E19B: DF 1E    stx  $1E
E19D: DE 20    ldx  $20
E19F: EB 00    addb $00,x
E1A1: CB 80    addb #$80
E1A3: F7 40 02 stb  $4002
E1A6: 7A 00 1B dec  $001B
E1A9: 26 0F    bne  $E1BA
E1AB: 96 19    lda  $19
E1AD: 97 1B    sta  $1B
E1AF: 08       inx  
E1B0: 8C 00 40 cmpx #$0040
E1B3: 26 03    bne  $E1B8
E1B5: CE 00 30 ldx  #$0030
E1B8: DF 20    stx  $20
E1BA: DE 22    ldx  $22
E1BC: 09       dex  
E1BD: DF 22    stx  $22
E1BF: 26 C4    bne  $E185
E1C1: 8D 02    bsr  $E1C5
E1C3: 20 C0    bra  $E185
E1C5: DE 1C    ldx  $1C
E1C7: DF 22    stx  $22
E1C9: CE 00 30 ldx  #$0030
E1CC: C6 10    ldb  #$10
E1CE: A6 00    lda  $00,x
E1D0: 27 03    beq  $E1D5
E1D2: 44       lsra 
E1D3: A7 00    sta  $00,x
E1D5: 08       inx  
E1D6: 5A       decb 
E1D7: 26 F5    bne  $E1CE
E1D9: 39       rts  
E1DA: 00       illegal
E1DB: 18       illegal
E1DC: 25 3B    bcs  $E219
E1DE: 3F       swi  
E1DF: 3B       rti  
E1E0: 25 18    bcs  $E1FA
E1E2: 00       illegal
E1E3: E8 DB    eorb $DB,x
E1E5: C5 C1    bitb #$C1
E1E7: C5 DB    bitb #$DB
E1E9: E8 86    eorb $86,x
E1EB: 01       nop  
E1EC: 97 1D    sta  $1D
E1EE: CE E2 AD ldx  #$E2AD
E1F1: DF 1E    stx  $1E
E1F3: CE 00 30 ldx  #$0030
E1F6: DF 20    stx  $20
E1F8: 86 10    lda  #$10
E1FA: BD E0 54 jsr  $E054
E1FD: 86 03    lda  #$03
E1FF: 20 15    bra  $E216
E201: 86 03    lda  #$03
E203: 97 1D    sta  $1D
E205: CE E2 AD ldx  #$E2AD
E208: DF 1E    stx  $1E
E20A: CE 00 30 ldx  #$0030
E20D: DF 20    stx  $20
E20F: 86 10    lda  #$10
E211: BD E0 54 jsr  $E054
E214: 86 0F    lda  #$0F
E216: 97 1C    sta  $1C
E218: CE 00 30 ldx  #$0030
E21B: A6 00    lda  $00,x
E21D: B7 40 02 sta  $4002
E220: 27 0A    beq  $E22C
E222: 4A       deca 
E223: A7 00    sta  $00,x
E225: 26 05    bne  $E22C
E227: 7A 00 1C dec  $001C
E22A: 27 0B    beq  $E237
E22C: 08       inx  
E22D: 8C 00 40 cmpx #$0040
E230: 26 E9    bne  $E21B
E232: CE 00 30 ldx  #$0030
E235: 20 E4    bra  $E21B
E237: 7A 00 1D dec  $001D
E23A: 26 C9    bne  $E205
E23C: 39       rts  
E23D: 86 03    lda  #$03
E23F: 97 1C    sta  $1C
E241: CE E2 AD ldx  #$E2AD
E244: DF 1E    stx  $1E
E246: CE 00 30 ldx  #$0030
E249: DF 20    stx  $20
E24B: 86 10    lda  #$10
E24D: BD E0 54 jsr  $E054
E250: CE 00 00 ldx  #$0000
E253: DF 20    stx  $20
E255: CE 00 30 ldx  #$0030
E258: A6 00    lda  $00,x
E25A: B7 40 02 sta  $4002
E25D: 08       inx  
E25E: 8C 00 40 cmpx #$0040
E261: 26 03    bne  $E266
E263: CE 00 30 ldx  #$0030
E266: DF 1E    stx  $1E
E268: CE 00 10 ldx  #$0010
E26B: 09       dex  
E26C: 26 FD    bne  $E26B
E26E: 86 80    lda  #$80
E270: B7 40 02 sta  $4002
E273: DE 20    ldx  $20
E275: 08       inx  
E276: 8C 00 80 cmpx #$0080
E279: 27 09    beq  $E284
E27B: DF 20    stx  $20
E27D: 09       dex  
E27E: 26 FD    bne  $E27D
E280: DE 1E    ldx  $1E
E282: 20 D4    bra  $E258
E284: 7A 00 1C dec  $001C
E287: 27 23    beq  $E2AC
E289: CE 00 30 ldx  #$0030
E28C: C6 10    ldb  #$10
E28E: A6 00    lda  $00,x
E290: 81 80    cmpa #$80
E292: 27 0A    beq  $E29E
E294: 22 05    bhi  $E29B
E296: 48       asla 
E297: 48       asla 
E298: 48       asla 
E299: 20 03    bra  $E29E
E29B: 44       lsra 
E29C: 44       lsra 
E29D: 44       lsra 
E29E: A7 00    sta  $00,x
E2A0: 08       inx  
E2A1: 5A       decb 
E2A2: 26 EA    bne  $E28E
E2A4: CE 30 00 ldx  #$3000
E2A7: 09       dex  
E2A8: 26 FD    bne  $E2A7
E2AA: 20 A4    bra  $E250
E2AC: 39       rts  
E2AD: 80 B1    suba #$B1
E2AF: DA F6    orb  $F6
E2B1: FF F6 DA stx  $F6DA
E2B4: B1 80 4F cmpa $804F
E2B7: 25 0A    bcs  $E2C3
E2B9: 00       illegal
E2BA: 0A       clv  
E2BB: 25 4F    bcs  $E30C
E2BD: 80 B1    suba #$B1
E2BF: DA F6    orb  $F6
E2C1: FF F6 DA stx  $F6DA
E2C4: B1 80 4F cmpa $804F
E2C7: 25 0A    bcs  $E2D3
E2C9: 00       illegal
E2CA: 0A       clv  
E2CB: 25 4F    bcs  $E31C
E2CD: 80 B1    suba #$B1
E2CF: DA F6    orb  $F6
E2D1: FF F6 DA stx  $F6DA
E2D4: B1 80 4F cmpa $804F
E2D7: 25 0A    bcs  $E2E3
E2D9: 00       illegal
E2DA: 0A       clv  
E2DB: 25 4F    bcs  $E32C
E2DD: 80 B1    suba #$B1
E2DF: DA F6    orb  $F6
E2E1: FF F6 DA stx  $F6DA
E2E4: B1 80 4F cmpa $804F
E2E7: 25 0A    bcs  $E2F3
E2E9: 00       illegal
E2EA: 0A       clv  
E2EB: 25 4F    bcs  $E33C
E2ED: CE E2 ED ldx  #$E2ED
E2F0: DF 1E    stx  $1E
E2F2: CE 00 30 ldx  #$0030
E2F5: DF 20    stx  $20
E2F7: 86 10    lda  #$10
E2F9: BD E0 54 jsr  $E054
E2FC: CE 00 FF ldx  #$00FF
E2FF: DF 20    stx  $20
E301: CE 00 30 ldx  #$0030
E304: A6 00    lda  $00,x
E306: DF 22    stx  $22
E308: 08       inx  
E309: 8C 00 40 cmpx #$0040
E30C: 26 03    bne  $E311
E30E: CE 00 30 ldx  #$0030
E311: AB 00    adda $00,x
E313: 44       lsra 
E314: B7 40 02 sta  $4002
E317: DF 1E    stx  $1E
E319: DE 22    ldx  $22
E31B: A7 00    sta  $00,x
E31D: DE 20    ldx  $20
E31F: 8C 00 00 cmpx #$0000
E322: 27 0A    beq  $E32E
E324: 09       dex  
E325: DF 20    stx  $20
E327: 09       dex  
E328: 26 FD    bne  $E327
E32A: DE 1E    ldx  $1E
E32C: 20 D6    bra  $E304
E32E: 39       rts  
E32F: CE E3 76 ldx  #$E376
E332: DF 1E    stx  $1E
E334: CE 00 30 ldx  #$0030
E337: DF 20    stx  $20
E339: 86 18    lda  #$18
E33B: BD E0 54 jsr  $E054
E33E: CE 01 44 ldx  #$0144
E341: DF 20    stx  $20
E343: CE 00 30 ldx  #$0030
E346: A6 00    lda  $00,x
E348: A1 0C    cmpa $0C,x
E34A: 27 0A    beq  $E356
E34C: 2B 05    bmi  $E353
E34E: 4A       deca 
E34F: A7 00    sta  $00,x
E351: 20 03    bra  $E356
E353: 4C       inca 
E354: A7 00    sta  $00,x
E356: B7 40 02 sta  $4002
E359: 08       inx  
E35A: 8C 00 3C cmpx #$003C
E35D: 26 03    bne  $E362
E35F: CE 00 30 ldx  #$0030
E362: DF 1E    stx  $1E
E364: DE 20    ldx  $20
E366: 8C 00 00 cmpx #$0000
E369: 27 0A    beq  $E375
E36B: 09       dex  
E36C: DF 20    stx  $20
E36E: 09       dex  
E36F: 26 FD    bne  $E36E
E371: DE 1E    ldx  $1E
E373: 20 D1    bra  $E346
E375: 39       rts  
E376: 80 FF    suba #$FF
E378: 80 00    suba #$00
E37A: 80 FF    suba #$FF
E37C: 80 00    suba #$00
E37E: 80 FF    suba #$FF
E380: 80 80    suba #$80
E382: 80 C0    suba #$C0
E384: DF FF    stx  $FF
E386: DF C0    stx  $C0
E388: 80 40    suba #$40
E38A: 11       cba  
E38B: 01       nop  
E38C: 11       cba  
E38D: 80 86    suba #$86
E38F: 18       illegal
E390: BD E7 B8 jsr  $E7B8
E393: 39       rts  
E394: 86 40    lda  #$40
E396: 97 0E    sta  $0E
E398: 86 40    lda  #$40
E39A: 97 11    sta  $11
E39C: 86 08    lda  #$08
E39E: 97 0F    sta  $0F
E3A0: 86 01    lda  #$01
E3A2: 97 12    sta  $12
E3A4: 86 03    lda  #$03
E3A6: 97 13    sta  $13
E3A8: 86 FF    lda  #$FF
E3AA: 97 0D    sta  $0D
E3AC: 86 40    lda  #$40
E3AE: 97 10    sta  $10
E3B0: CE E8 11 ldx  #$E811
E3B3: DF 1E    stx  $1E
E3B5: CE 00 30 ldx  #$0030
E3B8: DF 20    stx  $20
E3BA: DE 1E    ldx  $1E
E3BC: E6 00    ldb  $00,x
E3BE: 08       inx  
E3BF: DF 1E    stx  $1E
E3C1: DE 20    ldx  $20
E3C3: E7 00    stb  $00,x
E3C5: 08       inx  
E3C6: DF 20    stx  $20
E3C8: 8C 00 3C cmpx #$003C
E3CB: 26 ED    bne  $E3BA
E3CD: 96 0D    lda  $0D
E3CF: 97 14    sta  $14
E3D1: 96 10    lda  $10
E3D3: 97 15    sta  $15
E3D5: CE 00 30 ldx  #$0030
E3D8: E6 00    ldb  $00,x
E3DA: F7 40 02 stb  $4002
E3DD: 96 0F    lda  $0F
E3DF: 4A       deca 
E3E0: 26 FD    bne  $E3DF
E3E2: 08       inx  
E3E3: 8C 00 3C cmpx #$003C
E3E6: 26 F0    bne  $E3D8
E3E8: 96 14    lda  $14
E3EA: 91 0E    cmpa $0E
E3EC: 27 08    beq  $E3F6
E3EE: 7A 00 14 dec  $0014
E3F1: 4A       deca 
E3F2: 26 FD    bne  $E3F1
E3F4: 20 DF    bra  $E3D5
E3F6: CE 00 30 ldx  #$0030
E3F9: E6 00    ldb  $00,x
E3FB: F7 40 02 stb  $4002
E3FE: 96 12    lda  $12
E400: 4A       deca 
E401: 26 FD    bne  $E400
E403: 08       inx  
E404: 8C 00 3C cmpx #$003C
E407: 26 F0    bne  $E3F9
E409: 96 15    lda  $15
E40B: 91 11    cmpa $11
E40D: 27 08    beq  $E417
E40F: 7C 00 15 inc  $0015
E412: 4A       deca 
E413: 26 FD    bne  $E412
E415: 20 DF    bra  $E3F6
E417: 7A 00 13 dec  $0013
E41A: 26 B1    bne  $E3CD
E41C: 39       rts  
E41D: CE 00 80 ldx  #$0080
E420: DF 1E    stx  $1E
E422: 86 FF    lda  #$FF
E424: B7 40 02 sta  $4002
E427: DE 1E    ldx  $1E
E429: 8C 01 00 cmpx #$0100
E42C: 27 1E    beq  $E44C
E42E: 08       inx  
E42F: 08       inx  
E430: DF 1E    stx  $1E
E432: 09       dex  
E433: 26 FD    bne  $E432
E435: CE 00 80 ldx  #$0080
E438: 09       dex  
E439: 26 FD    bne  $E438
E43B: 7F 40 02 clr  $4002
E43E: 73 40 02 com  $4002
E441: 73 40 02 com  $4002
E444: 73 40 02 com  $4002
E447: 73 40 02 com  $4002
E44A: 20 D8    bra  $E424
E44C: 39       rts  
E44D: CE E4 94 ldx  #$E494
E450: DF 1E    stx  $1E
E452: CE 00 30 ldx  #$0030
E455: DF 20    stx  $20
E457: 86 18    lda  #$18
E459: BD E0 54 jsr  $E054
E45C: CE 00 00 ldx  #$0000
E45F: DF 20    stx  $20
E461: CE 00 30 ldx  #$0030
E464: A6 00    lda  $00,x
E466: A1 0C    cmpa $0C,x
E468: 27 0A    beq  $E474
E46A: 2B 05    bmi  $E471
E46C: 4A       deca 
E46D: A7 00    sta  $00,x
E46F: 20 03    bra  $E474
E471: 4C       inca 
E472: A7 00    sta  $00,x
E474: B7 40 02 sta  $4002
E477: 08       inx  
E478: 8C 00 3C cmpx #$003C
E47B: 26 03    bne  $E480
E47D: CE 00 30 ldx  #$0030
E480: DF 1E    stx  $1E
E482: DE 20    ldx  $20
E484: 8C 01 40 cmpx #$0140
E487: 27 0A    beq  $E493
E489: 08       inx  
E48A: DF 20    stx  $20
E48C: 09       dex  
E48D: 26 FD    bne  $E48C
E48F: DE 1E    ldx  $1E
E491: 20 D1    bra  $E464
E493: 39       rts  
E494: FF 80 00 stx  $8000
E497: 80 FF    suba #$FF
E499: 80 00    suba #$00
E49B: FF 80 00 stx  $8000
E49E: FF 80 80 stx  $8080
E4A1: C0 DF    subb #$DF
E4A3: FF DF C0 stx  $DFC0
E4A6: 80 40    suba #$40
E4A8: 11       cba  
E4A9: 01       nop  
E4AA: 11       cba  
E4AB: 40       nega 
E4AC: 96 17    lda  $17
E4AE: 27 0E    beq  $E4BE
E4B0: C6 02    ldb  #$02
E4B2: D7 0C    stb  $0C
E4B4: 81 01    cmpa #$01
E4B6: 26 09    bne  $E4C1
E4B8: 86 FC    lda  #$FC
E4BA: 97 16    sta  $16
E4BC: 20 07    bra  $E4C5
E4BE: 7F 00 0C clr  $000C
E4C1: 86 80    lda  #$80
E4C3: 97 16    sta  $16
E4C5: CE 00 30 ldx  #$0030
E4C8: 86 55    lda  #$55
E4CA: 97 01    sta  $01
E4CC: 5F       clrb 
E4CD: 96 01    lda  $01
E4CF: 44       lsra 
E4D0: 44       lsra 
E4D1: 44       lsra 
E4D2: 98 01    eora $01
E4D4: 44       lsra 
E4D5: 76 00 00 ror  $0000
E4D8: 76 00 01 ror  $0001
E4DB: 24 02    bcc  $E4DF
E4DD: D6 16    ldb  $16
E4DF: 96 01    lda  $01
E4E1: 84 03    anda #$03
E4E3: E7 00    stb  $00,x
E4E5: 08       inx  
E4E6: 8C 00 48 cmpx #$0048
E4E9: 27 05    beq  $E4F0
E4EB: 4A       deca 
E4EC: 26 F5    bne  $E4E3
E4EE: 20 DC    bra  $E4CC
E4F0: CE 00 30 ldx  #$0030
E4F3: 86 08    lda  #$08
E4F5: 97 02    sta  $02
E4F7: DF 1E    stx  $1E
E4F9: 5F       clrb 
E4FA: 4F       clra 
E4FB: AB 00    adda $00,x
E4FD: C9 00    adcb #$00
E4FF: 08       inx  
E500: 8C 00 48 cmpx #$0048
E503: 26 03    bne  $E508
E505: CE 00 30 ldx  #$0030
E508: 7A 00 02 dec  $0002
E50B: 26 EE    bne  $E4FB
E50D: 56       rorb 
E50E: 46       rora 
E50F: 56       rorb 
E510: 46       rora 
E511: 56       rorb 
E512: 46       rora 
E513: B7 40 02 sta  $4002
E516: DE 1E    ldx  $1E
E518: 7D 00 03 tst  $0003
E51B: 27 37    beq  $E554
E51D: 96 04    lda  $04
E51F: A7 00    sta  $00,x
E521: 7A 00 03 dec  $0003
E524: 08       inx  
E525: 8C 00 48 cmpx #$0048
E528: 26 1D    bne  $E547
E52A: CE 00 30 ldx  #$0030
E52D: 96 17    lda  $17
E52F: 81 01    cmpa #$01
E531: 26 0C    bne  $E53F
E533: 96 16    lda  $16
E535: 81 80    cmpa #$80
E537: 22 06    bhi  $E53F
E539: 7F 00 0C clr  $000C
E53C: 7F 00 17 clr  $0017
E53F: 96 16    lda  $16
E541: 90 0C    suba $0C
E543: 97 16    sta  $16
E545: 27 2A    beq  $E571
E547: DF 1E    stx  $1E
E549: CE 00 20 ldx  #$0020
E54C: 09       dex  
E54D: 26 FD    bne  $E54C
E54F: DE 1E    ldx  $1E
E551: 7E E4 F3 jmp  $E4F3
E554: 5F       clrb 
E555: 96 01    lda  $01
E557: 44       lsra 
E558: 44       lsra 
E559: 44       lsra 
E55A: 98 01    eora $01
E55C: 44       lsra 
E55D: 76 00 00 ror  $0000
E560: 76 00 01 ror  $0001
E563: 24 02    bcc  $E567
E565: D6 16    ldb  $16
E567: 96 01    lda  $01
E569: 84 03    anda #$03
E56B: 97 03    sta  $03
E56D: D7 04    stb  $04
E56F: 20 A7    bra  $E518
E571: 86 01    lda  #$01
E573: 97 17    sta  $17
E575: 39       rts  
E576: 86 02    lda  #$02
E578: 97 17    sta  $17
E57A: BD E4 AC jsr  $E4AC
E57D: 39       rts  
E57E: CE 00 30 ldx  #$0030
E581: 86 55    lda  #$55
E583: 97 01    sta  $01
E585: 86 FF    lda  #$FF
E587: 97 05    sta  $05
E589: 5F       clrb 
E58A: 96 01    lda  $01
E58C: 44       lsra 
E58D: 44       lsra 
E58E: 44       lsra 
E58F: 98 01    eora $01
E591: 44       lsra 
E592: 76 00 00 ror  $0000
E595: 76 00 01 ror  $0001
E598: 24 02    bcc  $E59C
E59A: D6 05    ldb  $05
E59C: 96 01    lda  $01
E59E: 84 03    anda #$03
E5A0: 8B 04    adda #$04
E5A2: E7 00    stb  $00,x
E5A4: 08       inx  
E5A5: 8C 00 48 cmpx #$0048
E5A8: 27 05    beq  $E5AF
E5AA: 4A       deca 
E5AB: 26 F5    bne  $E5A2
E5AD: 20 DA    bra  $E589
E5AF: CE 00 30 ldx  #$0030
E5B2: 86 08    lda  #$08
E5B4: 97 02    sta  $02
E5B6: DF 1E    stx  $1E
E5B8: 5F       clrb 
E5B9: 4F       clra 
E5BA: AB 00    adda $00,x
E5BC: C9 00    adcb #$00
E5BE: 08       inx  
E5BF: 8C 00 48 cmpx #$0048
E5C2: 26 03    bne  $E5C7
E5C4: CE 00 30 ldx  #$0030
E5C7: 7A 00 02 dec  $0002
E5CA: 26 EE    bne  $E5BA
E5CC: 56       rorb 
E5CD: 46       rora 
E5CE: 56       rorb 
E5CF: 46       rora 
E5D0: 56       rorb 
E5D1: 46       rora 
E5D2: B7 40 02 sta  $4002
E5D5: DE 1E    ldx  $1E
E5D7: 7D 00 03 tst  $0003
E5DA: 27 22    beq  $E5FE
E5DC: 96 04    lda  $04
E5DE: A7 00    sta  $00,x
E5E0: 7A 00 03 dec  $0003
E5E3: 08       inx  
E5E4: 8C 00 48 cmpx #$0048
E5E7: 26 08    bne  $E5F1
E5E9: CE 00 30 ldx  #$0030
E5EC: 7A 00 05 dec  $0005
E5EF: 27 2C    beq  $E61D
E5F1: DF 1E    stx  $1E
E5F3: CE 00 08 ldx  #$0008
E5F6: 09       dex  
E5F7: 26 FD    bne  $E5F6
E5F9: DE 1E    ldx  $1E
E5FB: 7E E5 B2 jmp  $E5B2
E5FE: 5F       clrb 
E5FF: 96 01    lda  $01
E601: 44       lsra 
E602: 44       lsra 
E603: 44       lsra 
E604: 98 01    eora $01
E606: 44       lsra 
E607: 76 00 00 ror  $0000
E60A: 76 00 01 ror  $0001
E60D: 24 02    bcc  $E611
E60F: D6 05    ldb  $05
E611: 96 01    lda  $01
E613: 84 03    anda #$03
E615: 8B 04    adda #$04
E617: 97 03    sta  $03
E619: D7 04    stb  $04
E61B: 20 BA    bra  $E5D7
E61D: 39       rts  
E61E: 86 17    lda  #$17
E620: BD E7 B8 jsr  $E7B8
E623: 4A       deca 
E624: 2A FA    bpl  $E620
E626: 39       rts  
E627: CE E6 3B ldx  #$E63B
E62A: A6 00    lda  $00,x
E62C: 81 FF    cmpa #$FF
E62E: 27 0A    beq  $E63A
E630: 08       inx  
E631: DF 22    stx  $22
E633: BD E7 B8 jsr  $E7B8
E636: DE 22    ldx  $22
E638: 20 F0    bra  $E62A
E63A: 39       rts  
E63B: 00       illegal
E63C: 03       illegal
E63D: 00       illegal
E63E: FF CE E6 stx  $CEE6
E641: 53       comb 
E642: A6 00    lda  $00,x
E644: 81 FF    cmpa #$FF
E646: 27 0A    beq  $E652
E648: 08       inx  
E649: DF 22    stx  $22
E64B: BD E7 B8 jsr  $E7B8
E64E: DE 22    ldx  $22
E650: 20 F0    bra  $E642
E652: 39       rts  
E653: 0C       clc  
E654: 0D       sec  
E655: 0E       cli  
E656: 0F       sei  
E657: 10       sba  
E658: 11       cba  
E659: FF CE E6 stx  $CEE6
E65C: 6E A6    jmp  $A6,x
E65E: 00       illegal
E65F: 81 FF    cmpa #$FF
E661: 27 0A    beq  $E66D
E663: 08       inx  
E664: DF 22    stx  $22
E666: BD E7 B8 jsr  $E7B8
E669: DE 22    ldx  $22
E66B: 20 F0    bra  $E65D
E66D: 39       rts  
E66E: 07       tpa  
E66F: 08       inx  
E670: 09       dex  
E671: 0A       clv  
E672: 0B       sev  
E673: 0C       clc  
E674: FF CE E6 stx  $CEE6
E677: 89 A6    adca #$A6
E679: 00       illegal
E67A: 81 FF    cmpa #$FF
E67C: 27 0A    beq  $E688
E67E: 08       inx  
E67F: DF 22    stx  $22
E681: BD E7 B8 jsr  $E7B8
E684: DE 22    ldx  $22
E686: 20 F0    bra  $E678
E688: 39       rts  
E689: 0E       cli  
E68A: 0F       sei  
E68B: 10       sba  
E68C: 0E       cli  
E68D: 0F       sei  
E68E: 10       sba  
E68F: 0E       cli  
E690: 0F       sei  
E691: 10       sba  
E692: 11       cba  
E693: FF CE E6 stx  $CEE6
E696: A8 A6    eora $A6,x
E698: 00       illegal
E699: 81 FF    cmpa #$FF
E69B: 27 0A    beq  $E6A7
E69D: 08       inx  
E69E: DF 22    stx  $22
E6A0: BD E7 B8 jsr  $E7B8
E6A3: DE 22    ldx  $22
E6A5: 20 F0    bra  $E697
E6A7: 39       rts  
E6A8: 07       tpa  
E6A9: 05       illegal
E6AA: 04       illegal
E6AB: 03       illegal
E6AC: 01       nop  
E6AD: 05       illegal
E6AE: 08       inx  
E6AF: 0B       sev  
E6B0: 0F       sei  
E6B1: 13       illegal
E6B2: 16       tab  
E6B3: FF CE E6 stx  $CEE6
E6B6: C8 A6    eorb #$A6
E6B8: 00       illegal
E6B9: 81 FF    cmpa #$FF
E6BB: 27 0A    beq  $E6C7
E6BD: 08       inx  
E6BE: DF 22    stx  $22
E6C0: BD E7 B8 jsr  $E7B8
E6C3: DE 22    ldx  $22
E6C5: 20 F0    bra  $E6B7
E6C7: 39       rts  
E6C8: 17       tba  
E6C9: 17       tba  
E6CA: 17       tba  
E6CB: 17       tba  
E6CC: 17       tba  
E6CD: 17       tba  
E6CE: 17       tba  
E6CF: 17       tba  
E6D0: 17       tba  
E6D1: 17       tba  
E6D2: FF CE E6 stx  $CEE6
E6D5: DF C6    stx  $C6
E6D7: 1C       illegal
E6D8: BD E7 98 jsr  $E798
E6DB: BD E6 FB jsr  $E6FB
E6DE: 39       rts  
E6DF: 30       tsx  
E6E0: 00       illegal
E6E1: 00       illegal
E6E2: 00       illegal
E6E3: 7F 00 00 clr  $0000
E6E6: 00       illegal
E6E7: 30       tsx  
E6E8: 00       illegal
E6E9: 00       illegal
E6EA: 00       illegal
E6EB: 01       nop  
E6EC: 00       illegal
E6ED: 00       illegal
E6EE: 00       illegal
E6EF: 7F 00 00 clr  $0000
E6F2: 00       illegal
E6F3: 02       illegal
E6F4: 00       illegal
E6F5: 00       illegal
E6F6: 00       illegal
E6F7: 01       nop  
E6F8: 00       illegal
E6F9: 00       illegal
E6FA: 00       illegal
E6FB: DF 51    stx  $51
E6FD: CE E7 B4 ldx  #$E7B4
E700: DF 53    stx  $53
E702: 86 80    lda  #$80
E704: D6 33    ldb  $33
E706: 2A 09    bpl  $E711
E708: D6 4D    ldb  $4D
E70A: 54       lsrb 
E70B: 54       lsrb 
E70C: 54       lsrb 
E70D: 5C       incb 
E70E: 5A       decb 
E70F: 26 FD    bne  $E70E
E711: 7A 00 38 dec  $0038
E714: 27 4C    beq  $E762
E716: 7A 00 39 dec  $0039
E719: 27 4C    beq  $E767
E71B: 7A 00 3A dec  $003A
E71E: 27 4C    beq  $E76C
E720: 7A 00 3B dec  $003B
E723: 26 DF    bne  $E704
E725: D6 33    ldb  $33
E727: 27 DB    beq  $E704
E729: C4 7F    andb #$7F
E72B: D7 3B    stb  $3B
E72D: D6 4D    ldb  $4D
E72F: 58       aslb 
E730: DB 4D    addb $4D
E732: CB 0B    addb #$0B
E734: D7 4D    stb  $4D
E736: 7A 00 4B dec  $004B
E739: 26 0E    bne  $E749
E73B: D6 3F    ldb  $3F
E73D: D7 4B    stb  $4B
E73F: DE 53    ldx  $53
E741: 09       dex  
E742: 8C E7 AD cmpx #$E7AD
E745: 27 4E    beq  $E795
E747: DF 53    stx  $53
E749: D6 4D    ldb  $4D
E74B: 2B 06    bmi  $E753
E74D: D4 37    andb $37
E74F: C4 7F    andb #$7F
E751: 20 05    bra  $E758
E753: D4 37    andb $37
E755: C4 7F    andb #$7F
E757: 50       negb 
E758: 36       psha 
E759: 1B       aba  
E75A: 16       tab  
E75B: 32       pula 
E75C: DE 53    ldx  $53
E75E: AD 00    jsr  $00,x
E760: 20 A2    bra  $E704
E762: CE 00 30 ldx  #$0030
E765: 20 08    bra  $E76F
E767: CE 00 31 ldx  #$0031
E76A: 20 03    bra  $E76F
E76C: CE 00 32 ldx  #$0032
E76F: 6D 18    tst  $18,x
E771: 27 12    beq  $E785
E773: 6A 18    dec  $18,x
E775: 26 0E    bne  $E785
E777: E6 0C    ldb  $0C,x
E779: E7 18    stb  $18,x
E77B: E6 00    ldb  $00,x
E77D: EB 10    addb $10,x
E77F: E1 14    cmpb $14,x
E781: 27 12    beq  $E795
E783: E7 00    stb  $00,x
E785: E6 00    ldb  $00,x
E787: E7 08    stb  $08,x
E789: AB 04    adda $04,x
E78B: 60 04    neg  $04,x
E78D: 16       tab  
E78E: DE 53    ldx  $53
E790: AD 00    jsr  $00,x
E792: 7E E7 04 jmp  $E704
E795: DE 51    ldx  $51
E797: 39       rts  
E798: 36       psha 
E799: A6 00    lda  $00,x
E79B: DF 51    stx  $51
E79D: DE 4F    ldx  $4F
E79F: A7 00    sta  $00,x
E7A1: 08       inx  
E7A2: DF 4F    stx  $4F
E7A4: DE 51    ldx  $51
E7A6: 08       inx  
E7A7: 5A       decb 
E7A8: 26 EF    bne  $E799
E7AA: 32       pula 
E7AB: 39       rts  
E7AC: 54       lsrb 
E7AD: 54       lsrb 
E7AE: 54       lsrb 
E7AF: 54       lsrb 
E7B0: 54       lsrb 
E7B1: 54       lsrb 
E7B2: 54       lsrb 
E7B3: 54       lsrb 
E7B4: F7 40 02 stb  $4002
E7B7: 39       rts  
E7B8: CE E8 11 ldx  #$E811
E7BB: DF 1E    stx  $1E
E7BD: CE 00 30 ldx  #$0030
E7C0: DF 20    stx  $20
E7C2: 36       psha 
E7C3: 86 0C    lda  #$0C
E7C5: BD E0 54 jsr  $E054
E7C8: 32       pula 
E7C9: 16       tab  
E7CA: CE E8 1D ldx  #$E81D
E7CD: 48       asla 
E7CE: BD E0 45 jsr  $E045
E7D1: EE 00    ldx  $00,x
E7D3: DF 06    stx  $06
E7D5: CE E8 4F ldx  #$E84F
E7D8: 17       tba  
E7D9: 48       asla 
E7DA: BD E0 45 jsr  $E045
E7DD: EE 00    ldx  $00,x
E7DF: DF 08    stx  $08
E7E1: CE 00 30 ldx  #$0030
E7E4: DF 0A    stx  $0A
E7E6: A6 00    lda  $00,x
E7E8: 81 80    cmpa #$80
E7EA: 22 07    bhi  $E7F3
E7EC: 27 08    beq  $E7F6
E7EE: 4C       inca 
E7EF: A7 00    sta  $00,x
E7F1: 20 03    bra  $E7F6
E7F3: 4A       deca 
E7F4: A7 00    sta  $00,x
E7F6: B7 40 02 sta  $4002
E7F9: DE 06    ldx  $06
E7FB: 09       dex  
E7FC: 26 FD    bne  $E7FB
E7FE: DE 0A    ldx  $0A
E800: 08       inx  
E801: 8C 00 3C cmpx #$003C
E804: 27 02    beq  $E808
E806: 20 DC    bra  $E7E4
E808: DE 08    ldx  $08
E80A: 09       dex  
E80B: DF 08    stx  $08
E80D: 26 D2    bne  $E7E1
E80F: 17       tba  
E810: 39       rts  
E811: 80 C0    suba #$C0
E813: DF FF    stx  $FF
E815: DF C0    stx  $C0
E817: 80 40    suba #$40
E819: 11       cba  
E81A: 00       illegal
E81B: 11       cba  
E81C: 40       nega 
E81D: 00       illegal
E81E: 20 00    bra  $E820
E820: 1F       illegal
E821: 00       illegal
E822: 1E       illegal
E823: 00       illegal
E824: 1D       illegal
E825: 00       illegal
E826: 1C       illegal
E827: 00       illegal
E828: 1B       aba  
E829: 00       illegal
E82A: 1A       illegal
E82B: 00       illegal
E82C: 19       daa  
E82D: 00       illegal
E82E: 18       illegal
E82F: 00       illegal
E830: 17       tba  
E831: 00       illegal
E832: 16       tab  
E833: 00       illegal
E834: 15       illegal
E835: 00       illegal
E836: 14       illegal
E837: 00       illegal
E838: 13       illegal
E839: 00       illegal
E83A: 12       illegal
E83B: 00       illegal
E83C: 11       cba  
E83D: 00       illegal
E83E: 10       sba  
E83F: 00       illegal
E840: 0F       sei  
E841: 00       illegal
E842: 0E       cli  
E843: 00       illegal
E844: 0D       sec  
E845: 00       illegal
E846: 0C       clc  
E847: 00       illegal
E848: 0B       sev  
E849: 00       illegal
E84A: 0A       clv  
E84B: 00       illegal
E84C: 09       dex  
E84D: 01       nop  
E84E: 00       illegal
E84F: 00       illegal
E850: 20 00    bra  $E852
E852: 20 00    bra  $E854
E854: 20 00    bra  $E856
E856: 20 00    bra  $E858
E858: 20 00    bra  $E85A
E85A: 20 00    bra  $E85C
E85C: 20 00    bra  $E85E
E85E: 20 00    bra  $E860
E860: 20 00    bra  $E862
E862: 20 00    bra  $E864
E864: 20 00    bra  $E866
E866: 20 00    bra  $E868
E868: 20 00    bra  $E86A
E86A: 20 00    bra  $E86C
E86C: 20 00    bra  $E86E
E86E: 20 00    bra  $E870
E870: 20 00    bra  $E872
E872: 20 00    bra  $E874
E874: 20 00    bra  $E876
E876: 20 00    bra  $E878
E878: 20 00    bra  $E87A
E87A: 20 00    bra  $E87C
E87C: 20 00    bra  $E87E
E87E: 20 00    bra  $E880
E880: 50       negb 
E881: CE F2 59 ldx  #$F259
E884: DF 57    stx  $57
E886: BD E8 A6 jsr  $E8A6
E889: 39       rts  
E88A: CE F2 1F ldx  #$F21F
E88D: DF 57    stx  $57
E88F: BD E8 A6 jsr  $E8A6
E892: 39       rts  
E893: CE F1 0D ldx  #$F10D
E896: DF 57    stx  $57
E898: BD E8 A6 jsr  $E8A6
E89B: 20 F6    bra  $E893
E89D: CE F1 0D ldx  #$F10D
E8A0: DF 57    stx  $57
E8A2: BD E8 A6 jsr  $E8A6
E8A5: 39       rts  
E8A6: DE 57    ldx  $57
E8A8: EE 00    ldx  $00,x
E8AA: 26 01    bne  $E8AD
E8AC: 39       rts  
E8AD: DF 59    stx  $59
E8AF: DE 57    ldx  $57
E8B1: 86 F0    lda  #$F0
E8B3: C6 A7    ldb  #$A7
E8B5: EB 02    addb $02,x
E8B7: 89 00    adca #$00
E8B9: 97 55    sta  $55
E8BB: D7 56    stb  $56
E8BD: DE 55    ldx  $55
E8BF: A6 00    lda  $00,x
E8C1: E6 01    ldb  $01,x
E8C3: 97 5B    sta  $5B
E8C5: D7 5C    stb  $5C
E8C7: 97 5D    sta  $5D
E8C9: D7 5E    stb  $5E
E8CB: A6 02    lda  $02,x
E8CD: E6 03    ldb  $03,x
E8CF: 97 5F    sta  $5F
E8D1: D7 60    stb  $60
E8D3: DE 57    ldx  $57
E8D5: 86 F0    lda  #$F0
E8D7: C6 A7    ldb  #$A7
E8D9: EB 03    addb $03,x
E8DB: 89 00    adca #$00
E8DD: 97 55    sta  $55
E8DF: D7 56    stb  $56
E8E1: DE 55    ldx  $55
E8E3: A6 00    lda  $00,x
E8E5: E6 01    ldb  $01,x
E8E7: 97 61    sta  $61
E8E9: D7 62    stb  $62
E8EB: 97 63    sta  $63
E8ED: D7 64    stb  $64
E8EF: A6 02    lda  $02,x
E8F1: E6 03    ldb  $03,x
E8F3: 97 65    sta  $65
E8F5: D7 66    stb  $66
E8F7: 96 57    lda  $57
E8F9: D6 58    ldb  $58
E8FB: CB 04    addb #$04
E8FD: 89 00    adca #$00
E8FF: 97 57    sta  $57
E901: D7 58    stb  $58
E903: DE 5B    ldx  $5B
E905: A6 00    lda  $00,x
E907: 08       inx  
E908: 9C 5F    cmpx $5F
E90A: 27 09    beq  $E915
E90C: C6 02    ldb  #$02
E90E: 5A       decb 
E90F: 26 FD    bne  $E90E
E911: D5 00    bitb $00
E913: 20 0C    bra  $E921
E915: DE 59    ldx  $59
E917: 09       dex  
E918: DF 59    stx  $59
E91A: 26 03    bne  $E91F
E91C: 7E E8 A6 jmp  $E8A6
E91F: DE 5D    ldx  $5D
E921: DF 5B    stx  $5B
E923: DE 61    ldx  $61
E925: AB 00    adda $00,x
E927: 8B 80    adda #$80
E929: B7 40 02 sta  $4002
E92C: 08       inx  
E92D: 9C 65    cmpx $65
E92F: 26 04    bne  $E935
E931: DE 63    ldx  $63
E933: 20 05    bra  $E93A
E935: C6 01    ldb  #$01
E937: 5A       decb 
E938: 26 FD    bne  $E937
E93A: DF 61    stx  $61
E93C: 7E E9 03 jmp  $E903
E93F: 00       illegal
E940: 03       illegal
E941: 06       tap  
E942: 09       dex  
E943: 0D       sec  
E944: 10       sba  
E945: 13       illegal
E946: 16       tab  
E947: 19       daa  
E948: 1C       illegal
E949: 1F       illegal
E94A: 22 25    bhi  $E971
E94C: 27 2A    beq  $E978
E94E: 2C 2E    bge  $E97E
E950: 31       ins  
E951: 33       pulb 
E952: 35       txs  
E953: 36       psha 
E954: 38       illegal
E955: 39       rts  
E956: 3B       rti  
E957: 3C       illegal
E958: 3D       illegal
E959: 3D       illegal
E95A: 3E       wai  
E95B: 3F       swi  
E95C: 3F       swi  
E95D: 3F       swi  
E95E: 3F       swi  
E95F: 3F       swi  
E960: 3E       wai  
E961: 3E       wai  
E962: 3D       illegal
E963: 3C       illegal
E964: 3B       rti  
E965: 3A       illegal
E966: 38       illegal
E967: 37       pshb 
E968: 35       txs  
E969: 33       pulb 
E96A: 31       ins  
E96B: 2F 2D    ble  $E99A
E96D: 2B 28    bmi  $E997
E96F: 26 23    bne  $E994
E971: 20 1D    bra  $E990
E973: 1A       illegal
E974: 17       tba  
E975: 14       illegal
E976: 11       cba  
E977: 0E       cli  
E978: 0B       sev  
E979: 07       tpa  
E97A: 04       illegal
E97B: 01       nop  
E97C: FD       illegal
E97D: F9 F6 F3 adcb $F6F3
E980: F0 ED E9 subb $EDE9
E983: E6 E3    ldb  $E3,x
E985: E1 DE    cmpb $DE,x
E987: DB D8    addb $D8
E989: D6 D3    ldb  $D3
E98B: D1 CF    cmpb $CF
E98D: CD       illegal
E98E: CB C9    addb #$C9
E990: C8 C6    eorb #$C6
E992: C5 C4    bitb #$C4
E994: C3       illegal
E995: C2 C1    sbcb #$C1
E997: C1 C0    cmpb #$C0
E999: C0 C0    subb #$C0
E99B: C0 C0    subb #$C0
E99D: C1 C2    cmpb #$C2
E99F: C2 C3    sbcb #$C3
E9A1: C5 C6    bitb #$C6
E9A3: C7 C9    stb  #$C9
E9A5: CB CD    addb #$CD
E9A7: CF D1 D3 stx  #$D1D3
E9AA: D5 D8    bitb $D8
E9AC: DA DD    orb  $DD
E9AE: E0 E3    subb $E3,x
E9B0: E6 E9    ldb  $E9,x
E9B2: EC       illegal
E9B3: EF F2    stx  $F2,x
E9B5: F5 F9 FC bitb $F9FC
E9B8: 00       illegal
E9B9: 03       illegal
E9BA: 06       tap  
E9BB: 0A       clv  
E9BC: 0D       sec  
E9BD: 11       cba  
E9BE: 14       illegal
E9BF: 17       tba  
E9C0: 1B       aba  
E9C1: 1E       illegal
E9C2: 21 24    brn  $E9E8
E9C4: 27 29    beq  $E9EF
E9C6: 2C 2E    bge  $E9F6
E9C8: 31       ins  
E9C9: 33       pulb 
E9CA: 35       txs  
E9CB: 37       pshb 
E9CC: 38       illegal
E9CD: 3A       illegal
E9CE: 3B       rti  
E9CF: 3C       illegal
E9D0: 3D       illegal
E9D1: 3E       wai  
E9D2: 3E       wai  
E9D3: 3F       swi  
E9D4: 3F       swi  
E9D5: 3F       swi  
E9D6: 3F       swi  
E9D7: 3E       wai  
E9D8: 3E       wai  
E9D9: 3D       illegal
E9DA: 3C       illegal
E9DB: 3B       rti  
E9DC: 3A       illegal
E9DD: 38       illegal
E9DE: 37       pshb 
E9DF: 35       txs  
E9E0: 33       pulb 
E9E1: 31       ins  
E9E2: 2E 2C    bgt  $EA10
E9E4: 29 27    bvs  $EA0D
E9E6: 24 21    bcc  $EA09
E9E8: 1E       illegal
E9E9: 1B       aba  
E9EA: 17       tba  
E9EB: 14       illegal
E9EC: 11       cba  
E9ED: 0D       sec  
E9EE: 0A       clv  
E9EF: 07       tpa  
E9F0: 03       illegal
E9F1: 00       illegal
E9F2: FB F8 F4 addb $F8F4
E9F5: F1 EE EA cmpb $EEEA
E9F8: E7 E4    stb  $E4,x
E9FA: E1 DE    cmpb $DE,x
E9FC: DB D8    addb $D8
E9FE: D5 D3    bitb $D3
EA00: D1 CE    cmpb $CE
EA02: CC       illegal
EA03: CA C8    orb  #$C8
EA05: C7 C5    stb  #$C5
EA07: C4 C3    andb #$C3
EA09: C2 C1    sbcb #$C1
EA0B: C1 C0    cmpb #$C0
EA0D: C0 C0    subb #$C0
EA0F: C0 C1    subb #$C1
EA11: C1 C2    cmpb #$C2
EA13: C3       illegal
EA14: C4 C5    andb #$C5
EA16: C7 C8    stb  #$C8
EA18: CA CC    orb  #$CC
EA1A: CE D0 D3 ldx  #$D0D3
EA1D: D5 D8    bitb $D8
EA1F: DB DE    addb $DE
EA21: E1 E4    cmpb $E4,x
EA23: E7 EA    stb  $EA,x
EA25: ED       illegal
EA26: F1 F4 F8 cmpb $F4F8
EA29: FB 00 03 addb $0003
EA2C: 07       tpa  
EA2D: 0B       sev  
EA2E: 0E       cli  
EA2F: 12       illegal
EA30: 15       illegal
EA31: 19       daa  
EA32: 1C       illegal
EA33: 1F       illegal
EA34: 22 26    bhi  $EA5C
EA36: 28 2B    bvc  $EA63
EA38: 2E 30    bgt  $EA6A
EA3A: 33       pulb 
EA3B: 35       txs  
EA3C: 37       pshb 
EA3D: 38       illegal
EA3E: 3A       illegal
EA3F: 3B       rti  
EA40: 3C       illegal
EA41: 3D       illegal
EA42: 3E       wai  
EA43: 3F       swi  
EA44: 3F       swi  
EA45: 3F       swi  
EA46: 3F       swi  
EA47: 3F       swi  
EA48: 3E       wai  
EA49: 3D       illegal
EA4A: 3C       illegal
EA4B: 3B       rti  
EA4C: 3A       illegal
EA4D: 38       illegal
EA4E: 36       psha 
EA4F: 34       des  
EA50: 32       pula 
EA51: 30       tsx  
EA52: 2D 2B    blt  $EA7F
EA54: 28 25    bvc  $EA7B
EA56: 22 1F    bhi  $EA77
EA58: 1B       aba  
EA59: 18       illegal
EA5A: 15       illegal
EA5B: 11       cba  
EA5C: 0E       cli  
EA5D: 0A       clv  
EA5E: 06       tap  
EA5F: 03       illegal
EA60: FE FA F7 ldx  $FAF7
EA63: F3       illegal
EA64: EF EC    stx  $EC,x
EA66: E8 E5    eorb $E5,x
EA68: E2 DF    sbcb $DF,x
EA6A: DB D9    addb $D9
EA6C: D6 D3    ldb  $D3
EA6E: D0 CE    subb $CE
EA70: CC       illegal
EA71: CA C8    orb  #$C8
EA73: C6 C5    ldb  #$C5
EA75: C3       illegal
EA76: C2 C1    sbcb #$C1
EA78: C1 C0    cmpb #$C0
EA7A: C0 C0    subb #$C0
EA7C: C0 C1    subb #$C1
EA7E: C1 C2    cmpb #$C2
EA80: C3       illegal
EA81: C4 C6    andb #$C6
EA83: C7 C9    stb  #$C9
EA85: CB CD    addb #$CD
EA87: CF D2 D4 stx  #$D2D4
EA8A: D7 DA    stb  $DA
EA8C: DD       illegal
EA8D: E0 E4    subb $E4,x
EA8F: E7 EA    stb  $EA,x
EA91: EE F1    ldx  $F1,x
EA93: F5 F9 00 bitb $F900
EA96: 03       illegal
EA97: 07       tpa  
EA98: 0B       sev  
EA99: 0F       sei  
EA9A: 13       illegal
EA9B: 17       tba  
EA9C: 1A       illegal
EA9D: 1E       illegal
EA9E: 21 24    brn  $EAC4
EAA0: 27 2A    beq  $EACC
EAA2: 2D 30    blt  $EAD4
EAA4: 32       pula 
EAA5: 35       txs  
EAA6: 37       pshb 
EAA7: 38       illegal
EAA8: 3A       illegal
EAA9: 3B       rti  
EAAA: 3D       illegal
EAAB: 3E       wai  
EAAC: 3E       wai  
EAAD: 3F       swi  
EAAE: 3F       swi  
EAAF: 3F       swi  
EAB0: 3F       swi  
EAB1: 3E       wai  
EAB2: 3D       illegal
EAB3: 3C       illegal
EAB4: 3B       rti  
EAB5: 3A       illegal
EAB6: 38       illegal
EAB7: 36       psha 
EAB8: 34       des  
EAB9: 32       pula 
EABA: 2F 2D    ble  $EAE9
EABC: 2A 27    bpl  $EAE5
EABE: 24 20    bcc  $EAE0
EAC0: 1D       illegal
EAC1: 19       daa  
EAC2: 16       tab  
EAC3: 12       illegal
EAC4: 0E       cli  
EAC5: 0A       clv  
EAC6: 07       tpa  
EAC7: 03       illegal
EAC8: FE FA F6 ldx  $FAF6
EACB: F2 EE EB sbcb $EEEB
EACE: E7 E4    stb  $E4,x
EAD0: E0 DD    subb $DD,x
EAD2: DA D7    orb  $D7
EAD4: D4 D1    andb $D1
EAD6: CE CC CA ldx  #$CCCA
EAD9: C8 C6    eorb #$C6
EADB: C5 C3    bitb #$C3
EADD: C2 C1    sbcb #$C1
EADF: C1 C0    cmpb #$C0
EAE1: C0 C0    subb #$C0
EAE3: C0 C1    subb #$C1
EAE5: C2 C3    sbcb #$C3
EAE7: C4 C5    andb #$C5
EAE9: C7 C9    stb  #$C9
EAEB: CB CD    addb #$CD
EAED: D0 D3    subb $D3
EAEF: D5 D8    bitb $D8
EAF1: DC       illegal
EAF2: DF E2    stx  $E2
EAF4: E6 E9    ldb  $E9,x
EAF6: ED       illegal
EAF7: F1 F5 F8 cmpb $F5F8
EAFA: 00       illegal
EAFB: 04       illegal
EAFC: 08       inx  
EAFD: 0C       clc  
EAFE: 10       sba  
EAFF: 14       illegal
EB00: 18       illegal
EB01: 1C       illegal
EB02: 1F       illegal
EB03: 23 26    bls  $EB2B
EB05: 29 2C    bvs  $EB33
EB07: 2F 32    ble  $EB3B
EB09: 34       des  
EB0A: 37       pshb 
EB0B: 38       illegal
EB0C: 3A       illegal
EB0D: 3C       illegal
EB0E: 3D       illegal
EB0F: 3E       wai  
EB10: 3E       wai  
EB11: 3F       swi  
EB12: 3F       swi  
EB13: 3F       swi  
EB14: 3E       wai  
EB15: 3E       wai  
EB16: 3D       illegal
EB17: 3C       illegal
EB18: 3A       illegal
EB19: 38       illegal
EB1A: 36       psha 
EB1B: 34       des  
EB1C: 32       pula 
EB1D: 2F 2C    ble  $EB4B
EB1F: 29 26    bvs  $EB47
EB21: 23 1F    bls  $EB42
EB23: 1B       aba  
EB24: 18       illegal
EB25: 14       illegal
EB26: 10       sba  
EB27: 0C       clc  
EB28: 08       inx  
EB29: 03       illegal
EB2A: FE FA F6 ldx  $FAF6
EB2D: F2 EE EA sbcb $EEEA
EB30: E6 E3    ldb  $E3,x
EB32: DF DC    stx  $DC
EB34: D8 D5    eorb $D5
EB36: D2 CF    sbcb $CF
EB38: CD       illegal
EB39: CA C8    orb  #$C8
EB3B: C6 C5    ldb  #$C5
EB3D: C3       illegal
EB3E: C2 C1    sbcb #$C1
EB40: C0 C0    subb #$C0
EB42: C0 C0    subb #$C0
EB44: C1 C1    cmpb #$C1
EB46: C2 C3    sbcb #$C3
EB48: C5 C7    bitb #$C7
EB4A: C9 CB    adcb #$CB
EB4C: CD       illegal
EB4D: D0 D3    subb $D3
EB4F: D6 D9    ldb  $D9
EB51: DC       illegal
EB52: E0 E3    subb $E3,x
EB54: E7 EB    stb  $EB,x
EB56: EF F3    stx  $F3,x
EB58: F7 FB 00 stb  $FB00
EB5B: 04       illegal
EB5C: 08       inx  
EB5D: 0D       sec  
EB5E: 11       cba  
EB5F: 15       illegal
EB60: 19       daa  
EB61: 1D       illegal
EB62: 21 25    brn  $EB89
EB64: 28 2B    bvc  $EB91
EB66: 2E 31    bgt  $EB99
EB68: 34       des  
EB69: 36       psha 
EB6A: 38       illegal
EB6B: 3A       illegal
EB6C: 3C       illegal
EB6D: 3D       illegal
EB6E: 3E       wai  
EB6F: 3F       swi  
EB70: 3F       swi  
EB71: 3F       swi  
EB72: 3F       swi  
EB73: 3E       wai  
EB74: 3D       illegal
EB75: 3C       illegal
EB76: 3B       rti  
EB77: 39       rts  
EB78: 37       pshb 
EB79: 35       txs  
EB7A: 32       pula 
EB7B: 2F 2C    ble  $EBA9
EB7D: 29 26    bvs  $EBA5
EB7F: 22 1E    bhi  $EB9F
EB81: 1A       illegal
EB82: 16       tab  
EB83: 12       illegal
EB84: 0E       cli  
EB85: 09       dex  
EB86: 05       illegal
EB87: 01       nop  
EB88: FB F7 F3 addb $F7F3
EB8B: EE EA    ldx  $EA,x
EB8D: E6 E2    ldb  $E2,x
EB8F: DE DB    ldx  $DB
EB91: D7 D4    stb  $D4
EB93: D1 CE    cmpb $CE
EB95: CC       illegal
EB96: C9 C7    adcb #$C7
EB98: C5 C4    bitb #$C4
EB9A: C2 C1    sbcb #$C1
EB9C: C1 C0    cmpb #$C0
EB9E: C0 C0    subb #$C0
EBA0: C1 C1    cmpb #$C1
EBA2: C3       illegal
EBA3: C4 C6    andb #$C6
EBA5: C7 CA    stb  #$CA
EBA7: CC       illegal
EBA8: CF D2 D5 stx  #$D2D5
EBAB: D8 DC    eorb $DC
EBAD: DF E3    stx  $E3
EBAF: E7 EB    stb  $EB,x
EBB1: EF F4    stx  $F4,x
EBB3: F8 00 04 eorb $0004
EBB6: 09       dex  
EBB7: 0D       sec  
EBB8: 12       illegal
EBB9: 16       tab  
EBBA: 1B       aba  
EBBB: 1F       illegal
EBBC: 23 27    bls  $EBE5
EBBE: 2A 2D    bpl  $EBED
EBC0: 31       ins  
EBC1: 33       pulb 
EBC2: 36       psha 
EBC3: 38       illegal
EBC4: 3A       illegal
EBC5: 3C       illegal
EBC6: 3D       illegal
EBC7: 3E       wai  
EBC8: 3F       swi  
EBC9: 3F       swi  
EBCA: 3F       swi  
EBCB: 3F       swi  
EBCC: 3E       wai  
EBCD: 3D       illegal
EBCE: 3B       rti  
EBCF: 3A       illegal
EBD0: 38       illegal
EBD1: 35       txs  
EBD2: 33       pulb 
EBD3: 30       tsx  
EBD4: 2D 29    blt  $EBFF
EBD6: 25 22    bcs  $EBFA
EBD8: 1E       illegal
EBD9: 19       daa  
EBDA: 15       illegal
EBDB: 11       cba  
EBDC: 0C       clc  
EBDD: 07       tpa  
EBDE: 03       illegal
EBDF: FD       illegal
EBE0: F9 F4 F0 adcb $F4F0
EBE3: EB E7    addb $E7,x
EBE5: E3       illegal
EBE6: DF DB    stx  $DB
EBE8: D7 D4    stb  $D4
EBEA: D0 CD    subb $CD
EBEC: CB C8    addb #$C8
EBEE: C6 C4    ldb  #$C4
EBF0: C3       illegal
EBF1: C2 C1    sbcb #$C1
EBF3: C0 C0    subb #$C0
EBF5: C0 C1    subb #$C1
EBF7: C1 C3    cmpb #$C3
EBF9: C4 C6    andb #$C6
EBFB: C8 CA    eorb #$CA
EBFD: CD       illegal
EBFE: D0 D3    subb $D3
EC00: D7 DA    stb  $DA
EC02: DE E2    ldx  $E2
EC04: E6 EA    ldb  $EA,x
EC06: EF F3    stx  $F3,x
EC08: F8 00 04 eorb $0004
EC0B: 09       dex  
EC0C: 0E       cli  
EC0D: 13       illegal
EC0E: 18       illegal
EC0F: 1C       illegal
EC10: 20 25    bra  $EC37
EC12: 28 2C    bvc  $EC40
EC14: 30       tsx  
EC15: 33       pulb 
EC16: 35       txs  
EC17: 38       illegal
EC18: 3A       illegal
EC19: 3C       illegal
EC1A: 3D       illegal
EC1B: 3E       wai  
EC1C: 3F       swi  
EC1D: 3F       swi  
EC1E: 3F       swi  
EC1F: 3E       wai  
EC20: 3D       illegal
EC21: 3C       illegal
EC22: 3B       rti  
EC23: 39       rts  
EC24: 36       psha 
EC25: 34       des  
EC26: 31       ins  
EC27: 2D 2A    blt  $EC53
EC29: 26 22    bne  $EC4D
EC2B: 1E       illegal
EC2C: 19       daa  
EC2D: 14       illegal
EC2E: 10       sba  
EC2F: 0B       sev  
EC30: 06       tap  
EC31: 01       nop  
EC32: FB F6 F2 addb $F6F2
EC35: ED       illegal
EC36: E8 E4    eorb $E4,x
EC38: DF DB    stx  $DB
EC3A: D7 D4    stb  $D4
EC3C: D0 CD    subb $CD
EC3E: CA C8    orb  #$C8
EC40: C6 C4    ldb  #$C4
EC42: C2 C1    sbcb #$C1
EC44: C0 C0    subb #$C0
EC46: C0 C0    subb #$C0
EC48: C1 C2    cmpb #$C2
EC4A: C4 C6    andb #$C6
EC4C: C8 CA    eorb #$CA
EC4E: CD       illegal
EC4F: D0 D4    subb $D4
EC51: D8 DB    eorb $DB
EC53: E0 E4    subb $E4,x
EC55: E8 ED    eorb $ED,x
EC57: F2 F7 00 sbcb $F700
EC5A: 05       illegal
EC5B: 0A       clv  
EC5C: 0F       sei  
EC5D: 14       illegal
EC5E: 19       daa  
EC5F: 1E       illegal
EC60: 22 26    bhi  $EC88
EC62: 2A 2E    bpl  $EC92
EC64: 32       pula 
EC65: 35       txs  
EC66: 37       pshb 
EC67: 3A       illegal
EC68: 3C       illegal
EC69: 3D       illegal
EC6A: 3E       wai  
EC6B: 3F       swi  
EC6C: 3F       swi  
EC6D: 3F       swi  
EC6E: 3E       wai  
EC6F: 3D       illegal
EC70: 3C       illegal
EC71: 3A       illegal
EC72: 37       pshb 
EC73: 35       txs  
EC74: 32       pula 
EC75: 2E 2B    bgt  $ECA2
EC77: 27 22    beq  $EC9B
EC79: 1E       illegal
EC7A: 19       daa  
EC7B: 14       illegal
EC7C: 0F       sei  
EC7D: 0A       clv  
EC7E: 05       illegal
EC7F: 00       illegal
EC80: FA F5 EF orb  $F5EF
EC83: EB E6    addb $E6,x
EC85: E1 DD    cmpb $DD,x
EC87: D8 D4    eorb $D4
EC89: D1 CD    cmpb $CD
EC8B: CA C8    orb  #$C8
EC8D: C5 C4    bitb #$C4
EC8F: C2 C1    sbcb #$C1
EC91: C0 C0    subb #$C0
EC93: C0 C1    subb #$C1
EC95: C2 C3    sbcb #$C3
EC97: C5 C7    bitb #$C7
EC99: CA CD    orb  #$CD
EC9B: D0 D4    subb $D4
EC9D: D8 DC    eorb $DC
EC9F: E0 E5    subb $E5,x
ECA1: EA EF    orb  $EF,x
ECA3: F4 F9 00 andb $F900
ECA6: 05       illegal
ECA7: 0B       sev  
ECA8: 10       sba  
ECA9: 15       illegal
ECAA: 1A       illegal
ECAB: 1F       illegal
ECAC: 24 28    bcc  $ECD6
ECAE: 2C 30    bge  $ECE0
ECB0: 34       des  
ECB1: 37       pshb 
ECB2: 39       rts  
ECB3: 3B       rti  
ECB4: 3D       illegal
ECB5: 3E       wai  
ECB6: 3F       swi  
ECB7: 3F       swi  
ECB8: 3F       swi  
ECB9: 3E       wai  
ECBA: 3D       illegal
ECBB: 3B       rti  
ECBC: 39       rts  
ECBD: 36       psha 
ECBE: 33       pulb 
ECBF: 30       tsx  
ECC0: 2C 28    bge  $ECEA
ECC2: 24 1F    bcc  $ECE3
ECC4: 1A       illegal
ECC5: 15       illegal
ECC6: 10       sba  
ECC7: 0A       clv  
ECC8: 05       illegal
ECC9: FE F9 F3 ldx  $F9F3
ECCC: EE E9    ldx  $E9,x
ECCE: E4 DF    andb $DF,x
ECD0: DA D6    orb  $D6
ECD2: D2 CE    sbcb $CE
ECD4: CB C8    addb #$C8
ECD6: C6 C4    ldb  #$C4
ECD8: C2 C1    sbcb #$C1
ECDA: C0 C0    subb #$C0
ECDC: C0 C1    subb #$C1
ECDE: C2 C4    sbcb #$C4
ECE0: C6 C9    ldb  #$C9
ECE2: CC       illegal
ECE3: CF D3 D7 stx  #$D3D7
ECE6: DB E0    addb $E0
ECE8: E5 EA    bitb $EA,x
ECEA: EF F4    stx  $F4,x
ECEC: FA 00 05 orb  $0005
ECEF: 0B       sev  
ECF0: 11       cba  
ECF1: 17       tba  
ECF2: 1C       illegal
ECF3: 21 26    brn  $ED1B
ECF5: 2A 2F    bpl  $ED26
ECF7: 32       pula 
ECF8: 36       psha 
ECF9: 38       illegal
ECFA: 3B       rti  
ECFB: 3D       illegal
ECFC: 3E       wai  
ECFD: 3F       swi  
ECFE: 3F       swi  
ECFF: 3F       swi  
ED00: 3E       wai  
ED01: 3C       illegal
ED02: 3B       rti  
ED03: 38       illegal
ED04: 35       txs  
ED05: 32       pula 
ED06: 2E 2A    bgt  $ED32
ED08: 25 21    bcs  $ED2B
ED0A: 1B       aba  
ED0B: 16       tab  
ED0C: 10       sba  
ED0D: 0B       sev  
ED0E: 05       illegal
ED0F: FE F8 F2 ldx  $F8F2
ED12: ED       illegal
ED13: E7 E2    stb  $E2,x
ED15: DD       illegal
ED16: D8 D4    eorb $D4
ED18: D0 CC    subb $CC
ED1A: C9 C6    adcb #$C6
ED1C: C4 C2    andb #$C2
ED1E: C1 C0    cmpb #$C0
ED20: C0 C0    subb #$C0
ED22: C1 C3    cmpb #$C3
ED24: C5 C7    bitb #$C7
ED26: CA CD    orb  #$CD
ED28: D1 D5    cmpb $D5
ED2A: DA DE    orb  $DE
ED2C: E4 E9    andb $E9,x
ED2E: EE F4    ldx  $F4,x
ED30: FA 00 06 orb  $0006
ED33: 0C       clc  
ED34: 12       illegal
ED35: 18       illegal
ED36: 1D       illegal
ED37: 23 28    bls  $ED61
ED39: 2C 31    bge  $ED6C
ED3B: 34       des  
ED3C: 38       illegal
ED3D: 3A       illegal
ED3E: 3C       illegal
ED3F: 3E       wai  
ED40: 3F       swi  
ED41: 3F       swi  
ED42: 3F       swi  
ED43: 3E       wai  
ED44: 3C       illegal
ED45: 3A       illegal
ED46: 38       illegal
ED47: 34       des  
ED48: 31       ins  
ED49: 2C 28    bge  $ED73
ED4B: 23 1D    bls  $ED6A
ED4D: 18       illegal
ED4E: 12       illegal
ED4F: 0C       clc  
ED50: 06       tap  
ED51: 00       illegal
ED52: F8 F2 EC eorb $F2EC
ED55: E7 E1    stb  $E1,x
ED57: DC       illegal
ED58: D7 D2    stb  $D2
ED5A: CE CB C7 ldx  #$CBC7
ED5D: C5 C3    bitb #$C3
ED5F: C1 C0    cmpb #$C0
ED61: C0 C0    subb #$C0
ED63: C1 C3    cmpb #$C3
ED65: C5 C7    bitb #$C7
ED67: CB CE    addb #$CE
ED69: D2 D7    sbcb $D7
ED6B: DC       illegal
ED6C: E1 E7    cmpb $E7,x
ED6E: EC       illegal
ED6F: F2 F8 00 sbcb $F800
ED72: 06       tap  
ED73: 0D       sec  
ED74: 13       illegal
ED75: 19       daa  
ED76: 1F       illegal
ED77: 25 2A    bcs  $EDA3
ED79: 2E 33    bgt  $EDAE
ED7B: 36       psha 
ED7C: 39       rts  
ED7D: 3C       illegal
ED7E: 3D       illegal
ED7F: 3F       swi  
ED80: 3F       swi  
ED81: 3F       swi  
ED82: 3E       wai  
ED83: 3C       illegal
ED84: 3A       illegal
ED85: 37       pshb 
ED86: 33       pulb 
ED87: 2F 2B    ble  $EDB4
ED89: 26 20    bne  $EDAB
ED8B: 1A       illegal
ED8C: 14       illegal
ED8D: 0E       cli  
ED8E: 07       tpa  
ED8F: 01       nop  
ED90: F9 F3 ED adcb $F3ED
ED93: E6 E1    ldb  $E1,x
ED95: DB D6    addb $D6
ED97: D1 CD    cmpb $CD
ED99: C9 C6    adcb #$C6
ED9B: C4 C2    andb #$C2
ED9D: C1 C0    cmpb #$C0
ED9F: C0 C1    subb #$C1
EDA1: C2 C5    sbcb #$C5
EDA3: C7 CB    stb  #$CB
EDA5: CF D3 D8 stx  #$D3D8
EDA8: DD       illegal
EDA9: E3       illegal
EDAA: E9 EF    adcb $EF,x
EDAC: F5 00 06 bitb $0006
EDAF: 0D       sec  
EDB0: 14       illegal
EDB1: 1B       aba  
EDB2: 21 27    brn  $EDDB
EDB4: 2C 31    bge  $EDE7
EDB6: 35       txs  
EDB7: 38       illegal
EDB8: 3B       rti  
EDB9: 3D       illegal
EDBA: 3E       wai  
EDBB: 3F       swi  
EDBC: 3F       swi  
EDBD: 3E       wai  
EDBE: 3C       illegal
EDBF: 3A       illegal
EDC0: 37       pshb 
EDC1: 33       pulb 
EDC2: 2E 29    bgt  $EDED
EDC4: 24 1E    bcc  $EDE4
EDC6: 17       tba  
EDC7: 11       cba  
EDC8: 0A       clv  
EDC9: 03       illegal
EDCA: FB F4 EE addb $F4EE
EDCD: E7 E1    stb  $E1,x
EDCF: DB D5    addb $D5
EDD1: D1 CC    cmpb $CC
EDD3: C8 C5    eorb #$C5
EDD5: C3       illegal
EDD6: C1 C0    cmpb #$C0
EDD8: C0 C1    subb #$C1
EDDA: C2 C4    sbcb #$C4
EDDC: C7 CA    stb  #$CA
EDDE: CE D3 D8 ldx  #$D3D8
EDE1: DE E4    ldx  $E4
EDE3: EA F1    orb  $F1,x
EDE5: F8 00 07 eorb $0007
EDE8: 0E       cli  
EDE9: 15       illegal
EDEA: 1C       illegal
EDEB: 23 28    bls  $EE15
EDED: 2E 33    bgt  $EE22
EDEF: 37       pshb 
EDF0: 3A       illegal
EDF1: 3C       illegal
EDF2: 3E       wai  
EDF3: 3F       swi  
EDF4: 3F       swi  
EDF5: 3E       wai  
EDF6: 3C       illegal
EDF7: 3A       illegal
EDF8: 36       psha 
EDF9: 32       pula 
EDFA: 2D 28    blt  $EE24
EDFC: 22 1C    bhi  $EE1A
EDFE: 15       illegal
EDFF: 0E       cli  
EE00: 06       tap  
EE01: FE F7 EF ldx  $F7EF
EE04: E8 E2    eorb $E2,x
EE06: DB D6    addb $D6
EE08: D0 CC    subb $CC
EE0A: C8 C5    eorb #$C5
EE0C: C2 C1    sbcb #$C1
EE0E: C0 C0    subb #$C0
EE10: C1 C3    cmpb #$C3
EE12: C5 C9    bitb #$C9
EE14: CD       illegal
EE15: D2 D7    sbcb $D7
EE17: DD       illegal
EE18: E4 EA    andb $EA,x
EE1A: F1 F9 00 cmpb $F900
EE1D: 07       tpa  
EE1E: 0F       sei  
EE1F: 17       tba  
EE20: 1E       illegal
EE21: 24 2A    bcc  $EE4D
EE23: 30       tsx  
EE24: 35       txs  
EE25: 38       illegal
EE26: 3B       rti  
EE27: 3E       wai  
EE28: 3F       swi  
EE29: 3F       swi  
EE2A: 3E       wai  
EE2B: 3C       illegal
EE2C: 3A       illegal
EE2D: 36       psha 
EE2E: 32       pula 
EE2F: 2D 27    blt  $EE58
EE31: 20 19    bra  $EE4C
EE33: 12       illegal
EE34: 0A       clv  
EE35: 03       illegal
EE36: FA F2 EB orb  $F2EB
EE39: E4 DD    andb $DD,x
EE3B: D7 D1    stb  $D1
EE3D: CC       illegal
EE3E: C8 C5    eorb #$C5
EE40: C2 C1    sbcb #$C1
EE42: C0 C0    subb #$C0
EE44: C2 C4    sbcb #$C4
EE46: C7 CB    stb  #$CB
EE48: D0 D5    subb $D5
EE4A: DC       illegal
EE4B: E2 E9    sbcb $E9,x
EE4D: F1 F8 00 cmpb $F800
EE50: 08       inx  
EE51: 10       sba  
EE52: 18       illegal
EE53: 1F       illegal
EE54: 26 2C    bne  $EE82
EE56: 32       pula 
EE57: 37       pshb 
EE58: 3A       illegal
EE59: 3D       illegal
EE5A: 3E       wai  
EE5B: 3F       swi  
EE5C: 3E       wai  
EE5D: 3D       illegal
EE5E: 3A       illegal
EE5F: 36       psha 
EE60: 32       pula 
EE61: 2C 26    bge  $EE89
EE63: 1F       illegal
EE64: 18       illegal
EE65: 10       sba  
EE66: 08       inx  
EE67: FE F6 EE ldx  $F6EE
EE6A: E6 DF    ldb  $DF,x
EE6C: D8 D2    eorb $D2
EE6E: CD       illegal
EE6F: C8 C5    eorb #$C5
EE71: C2 C0    sbcb #$C0
EE73: C0 C1    subb #$C1
EE75: C2 C5    sbcb #$C5
EE77: C9 CD    adcb #$CD
EE79: D3       illegal
EE7A: D9 E0    adcb $E0
EE7C: E7 EF    stb  $EF,x
EE7E: F7 00 08 stb  $0008
EE81: 11       cba  
EE82: 19       daa  
EE83: 21 28    brn  $EEAD
EE85: 2E 34    bgt  $EEBB
EE87: 38       illegal
EE88: 3C       illegal
EE89: 3E       wai  
EE8A: 3F       swi  
EE8B: 3F       swi  
EE8C: 3D       illegal
EE8D: 3B       rti  
EE8E: 37       pshb 
EE8F: 32       pula 
EE90: 2C 26    bge  $EEB8
EE92: 1E       illegal
EE93: 16       tab  
EE94: 0E       cli  
EE95: 05       illegal
EE96: FB F3 EA addb $F3EA
EE99: E2 DB    sbcb $DB,x
EE9B: D4 CE    andb $CE
EE9D: C9 C5    adcb #$C5
EE9F: C2 C1    sbcb #$C1
EEA1: C0 C1    subb #$C1
EEA3: C3       illegal
EEA4: C6 CA    ldb  #$CA
EEA6: CF D5 DC stx  #$D5DC
EEA9: E3       illegal
EEAA: EB F4    addb $F4,x
EEAC: 00       illegal
EEAD: 09       dex  
EEAE: 12       illegal
EEAF: 1B       aba  
EEB0: 23 2A    bls  $EEDC
EEB2: 31       ins  
EEB3: 36       psha 
EEB4: 3A       illegal
EEB5: 3D       illegal
EEB6: 3F       swi  
EEB7: 3F       swi  
EEB8: 3E       wai  
EEB9: 3B       rti  
EEBA: 38       illegal
EEBB: 33       pulb 
EEBC: 2D 25    blt  $EEE3
EEBE: 1E       illegal
EEBF: 15       illegal
EEC0: 0C       clc  
EEC1: 03       illegal
EEC2: F9 F0 E7 adcb $F0E7
EEC5: DF D7    stx  $D7
EEC7: D0 CB    subb $CB
EEC9: C6 C3    ldb  #$C3
EECB: C1 C0    cmpb #$C0
EECD: C1 C3    cmpb #$C3
EECF: C6 CA    ldb  #$CA
EED1: D0 D7    subb $D7
EED3: DE E6    ldx  $E6
EED5: EF F8    stx  $F8,x
EED7: 00       illegal
EED8: 09       dex  
EED9: 13       illegal
EEDA: 1C       illegal
EEDB: 25 2C    bcs  $EF09
EEDD: 33       pulb 
EEDE: 38       illegal
EEDF: 3C       illegal
EEE0: 3E       wai  
EEE1: 3F       swi  
EEE2: 3E       wai  
EEE3: 3C       illegal
EEE4: 39       rts  
EEE5: 34       des  
EEE6: 2D 26    blt  $EF0E
EEE8: 1E       illegal
EEE9: 14       illegal
EEEA: 0B       sev  
EEEB: 01       nop  
EEEC: F6 ED E4 ldb  $EDE4
EEEF: DB D4    addb $D4
EEF1: CD       illegal
EEF2: C8 C4    eorb #$C4
EEF4: C1 C0    cmpb #$C0
EEF6: C0 C2    subb #$C2
EEF8: C6 CA    ldb  #$CA
EEFA: D0 D8    subb $D8
EEFC: E0 E8    subb $E8,x
EEFE: F2 00 0A sbcb $000A
EF01: 14       illegal
EF02: 1E       illegal
EF03: 27 2E    beq  $EF33
EF05: 35       txs  
EF06: 3A       illegal
EF07: 3D       illegal
EF08: 3F       swi  
EF09: 3F       swi  
EF0A: 3D       illegal
EF0B: 3A       illegal
EF0C: 35       txs  
EF0D: 2E 27    bgt  $EF36
EF0F: 1E       illegal
EF10: 14       illegal
EF11: 0A       clv  
EF12: 00       illegal
EF13: F4 EA E1 andb $EAE1
EF16: D8 D1    eorb $D1
EF18: CA C5    orb  #$C5
EF1A: C2 C0    sbcb #$C0
EF1C: C0 C2    subb #$C2
EF1E: C5 CA    bitb #$CA
EF20: D0 D8    subb $D8
EF22: E0 EA    subb $EA,x
EF24: F4 00 0B andb $000B
EF27: 15       illegal
EF28: 1F       illegal
EF29: 28 30    bvc  $EF5B
EF2B: 37       pshb 
EF2C: 3B       rti  
EF2D: 3E       wai  
EF2E: 3F       swi  
EF2F: 3E       wai  
EF30: 3B       rti  
EF31: 36       psha 
EF32: 30       tsx  
EF33: 28 1F    bvc  $EF54
EF35: 15       illegal
EF36: 0A       clv  
EF37: FE F3 E9 ldx  $F3E9
EF3A: DF D6    stx  $D6
EF3C: CE C8 C4 ldx  #$C8C4
EF3F: C1 C0    cmpb #$C0
EF41: C1 C4    cmpb #$C4
EF43: C9 CF    adcb #$CF
EF45: D7 E0    stb  $E0
EF47: EA F4    orb  $F4,x
EF49: 00       illegal
EF4A: 0B       sev  
EF4B: 17       tba  
EF4C: 21 2B    brn  $EF79
EF4E: 32       pula 
EF4F: 39       rts  
EF50: 3D       illegal
EF51: 3F       swi  
EF52: 3F       swi  
EF53: 3D       illegal
EF54: 38       illegal
EF55: 32       pula 
EF56: 2A 21    bpl  $EF79
EF58: 16       tab  
EF59: 0B       sev  
EF5A: FE F2 E7 ldx  $F2E7
EF5D: DD       illegal
EF5E: D4 CC    andb $CC
EF60: C6 C2    ldb  #$C2
EF62: C0 C0    subb #$C0
EF64: C2 C7    sbcb #$C7
EF66: CD       illegal
EF67: D5 DE    bitb $DE
EF69: E9 F4    adcb $F4,x
EF6B: 00       illegal
EF6C: 0C       clc  
EF6D: 18       illegal
EF6E: 23 2C    bls  $EF9C
EF70: 34       des  
EF71: 3A       illegal
EF72: 3E       wai  
EF73: 3F       swi  
EF74: 3E       wai  
EF75: 3A       illegal
EF76: 34       des  
EF77: 2C 23    bge  $EF9C
EF79: 18       illegal
EF7A: 0C       clc  
EF7B: 00       illegal
EF7C: F2 E7 DC sbcb $E7DC
EF7F: D2 CB    sbcb $CB
EF81: C5 C1    bitb #$C1
EF83: C0 C1    subb #$C1
EF85: C5 CB    bitb #$CB
EF87: D2 DC    sbcb $DC
EF89: E7 F2    stb  $F2,x
EF8B: 00       illegal
EF8C: 0D       sec  
EF8D: 19       daa  
EF8E: 25 2F    bcs  $EFBF
EF90: 36       psha 
EF91: 3C       illegal
EF92: 3F       swi  
EF93: 3F       swi  
EF94: 3C       illegal
EF95: 37       pshb 
EF96: 2F 26    ble  $EFBE
EF98: 1A       illegal
EF99: 0E       cli  
EF9A: 01       nop  
EF9B: F3       illegal
EF9C: E6 DB    ldb  $DB,x
EF9E: D1 C9    cmpb $C9
EFA0: C4 C0    andb #$C0
EFA2: C0 C2    subb #$C2
EFA4: C7 CE    stb  #$CE
EFA6: D8 E3    eorb $E3
EFA8: EF 00    stx  $00,x
EFAA: 0D       sec  
EFAB: 1B       aba  
EFAC: 27 31    beq  $EFDF
EFAE: 38       illegal
EFAF: 3D       illegal
EFB0: 3F       swi  
EFB1: 3E       wai  
EFB2: 3A       illegal
EFB3: 33       pulb 
EFB4: 29 1E    bvs  $EFD4
EFB6: 11       cba  
EFB7: 03       illegal
EFB8: F4 E7 DB andb $E7DB
EFBB: D1 C8    cmpb $C8
EFBD: C3       illegal
EFBE: C0 C1    subb #$C1
EFC0: C4 CA    andb #$CA
EFC2: D3       illegal
EFC3: DE EA    ldx  $EA
EFC5: 00       illegal
EFC6: 0E       cli  
EFC7: 1C       illegal
EFC8: 28 33    bvc  $EFFD
EFCA: 3A       illegal
EFCB: 3E       wai  
EFCC: 3F       swi  
EFCD: 3C       illegal
EFCE: 36       psha 
EFCF: 2D 22    blt  $EFF3
EFD1: 15       illegal
EFD2: 06       tap  
EFD3: F7 E8 DB stb  $E8DB
EFD6: D0 C8    subb $C8
EFD8: C2 C0    sbcb #$C0
EFDA: C1 C5    cmpb #$C5
EFDC: CD       illegal
EFDD: D7 E4    stb  $E4
EFDF: F1 00 0F cmpb $000F
EFE2: 1E       illegal
EFE3: 2A 35    bpl  $F01A
EFE5: 3B       rti  
EFE6: 3F       swi  
EFE7: 3E       wai  
EFE8: 3A       illegal
EFE9: 32       pula 
EFEA: 27 19    beq  $F005
EFEC: 0A       clv  
EFED: FA EB DD orb  $EBDD
EFF0: D1 C8    cmpb $C8
EFF2: C2 C0    sbcb #$C0
EFF4: C2 C7    sbcb #$C7
EFF6: D0 DC    subb $DC
EFF8: E9 00    adcb $00,x
EFFA: 10       sba  
EFFB: 1F       illegal
EFFC: 2C 37    bge  $F035
EFFE: 3D       illegal
EFFF: 3F       swi  
