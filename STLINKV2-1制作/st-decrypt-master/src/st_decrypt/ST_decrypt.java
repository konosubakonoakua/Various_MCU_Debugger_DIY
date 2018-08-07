package st_decrypt;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.apache.commons.cli.*;

class Dumper {

    private static FileOutputStream fstream;

    public Dumper(String filename) {
        try {
            fstream = new FileOutputStream(filename);
        } catch (FileNotFoundException ex) {
            Logger.getLogger(Dumper.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    /**
     * Dump byte array to a file
     *
     * @param dump byte array
     * @param filename
     */
    static void dump(byte[] dump, String filename) {
        if (dump == null) {
            return;
        }
        FileOutputStream fos = null;
        try {
            fos = new FileOutputStream(filename);
        } catch (FileNotFoundException ex) {
            Logger.getLogger(ST_decrypt.class.getName()).log(Level.SEVERE, null, ex);
        }
        try {
            fos.write(dump, 0, dump.length);
            fos.flush();
            fos.close();
        } catch (IOException ex) {
            Logger.getLogger(ST_decrypt.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    void append(byte[] b) {
        try {
            fstream.write(b);
        } catch (IOException ex) {
            Logger.getLogger(Dumper.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    void close() {
        try {
            fstream.close();
        } catch (IOException ex) {
            Logger.getLogger(Dumper.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
}

class Crypto {

    private static int d(final int n) {
        final long n2 = n & 0xFFFFFFFFL;
        return (int) ((n2 & 0x7F7F7F7FL) << 1 ^ ((n2 & 0xFFFFFFFF80808080L) >> 7) * 27L);
    }

    private static int a(final int n, final int n2) {
        final long n3 = n & 0xFFFFFFFFL;
        return (int) (n3 >> n2 * 8 | n3 << 32 - n2 * 8);
    }

    static int polynom2(int n) {
        final int d3;
        final int d2;
        final int d = d(d2 = d(d3 = d(n)));
        n ^= d;
        return d3 ^ (d2 ^ d ^ a(d3 ^ n, 3) ^ a(d2 ^ n, 2) ^ a(n, 1));
    }

    static int polynom(int n) {
        return (d(n) ^ a(n ^ d(n), 3) ^ a(n, 2) ^ a(n, 1));
    }
}

public class ST_decrypt {

    /**
     * Encrypt or decrypt input file
     */
    private static boolean encrypt;

    private static String encryptionKey;

    private static Dumper dumper;

    public static void main(String[] args) {
        CommandLineParser parser = new DefaultParser();
        Options options = new Options();
        String help = "st_decrypt.jar [-e] -k <key> -i <input> -o <output>";
        options.addOption("k", "key", true, "encryption key");
        options.addOption("e", "encrypt", false, "encrypt binary");
        options.addOption("i", "input", true, "input file");
        options.addOption("o", "output", true, "output file");
        HelpFormatter formatter = new HelpFormatter();
        CommandLine opts = null;
        try {
            opts = parser.parse(options, args);
            if (opts.hasOption("key")
                    && opts.hasOption("input")
                    && opts.hasOption("output")) {
                encryptionKey = opts.getOptionValue("key");
            } else {
                formatter.printHelp(help, options);
                System.exit(1);
            }
            encrypt = opts.hasOption("encrypt");
        } catch (ParseException exp) {
            System.out.println(exp.getMessage());
            formatter.printHelp(help, options);
            System.exit(1);
        }

        dumper = new Dumper(opts.getOptionValue("output"));
        dump_fw(opts.getOptionValue("input"));
        dumper.close();
        System.out.println("Done!");
    }

    static long getFileLen(String file) {
        long n2 = 0L;
        FileInputStream fis = null;
        try {
            File f = new File(file);
            fis = new FileInputStream(f);
        } catch (Exception ex) {
        }
        try {
            BufferedInputStream bufferedInputStream = new BufferedInputStream(fis);
            while (true) {
                int read;
                try {
                    read = bufferedInputStream.read();
                } catch (IOException ex) {
                    System.out.println("Failure opening file");
                    read = -1;
                }
                if (read == -1) {
                    break;
                }
                ++n2;
            }
            bufferedInputStream.close();
        } catch (IOException ex3) {
            System.out.println("Failure getting firmware data");
        }
        return n2;
    }

    /**
     *
     * @param array firmware byte array
     * @param n length
     * @param n2 ??
     * @return
     */
    static long encodeAndWrite(final byte[] array, long n, final int n2) {
        long a = 0L;
        final byte[] array2 = new byte[4 * ((array.length + 3) / 4)];
        final byte[] array3 = new byte[16];
        str_to_arr(encryptionKey, array3);

        if (encrypt) {
            encrypt(array, array2, array3, array.length);
        } else {
            decrypt(array, array2, array3, array.length);
        }

        /* Send chunk of data to device */
        dumper.append(array2);
        return a;
    }

    static long writeFirmware(final BufferedInputStream bufferedInputStream, final long n) {
        long a = 0L;
        final byte[] array = new byte[3072];
        long n4 = 0L;
        try {
            while (n4 < n && a == 0L) {
                final long n5;
                if ((n5 = bufferedInputStream.read(array)) != -1L) {
                    encodeAndWrite(array, n4 + 134234112L, 3072);
                    n4 += n5;
                }
            }
        } catch (IOException ex) {
            System.out.println("Failure reading file: " + ex.getMessage());
            System.exit(1);
        }
        return a;
    }

    static void dump_fw(String file) {
        FileInputStream fis = null;
        try {
            File f = new File(file);
            fis = new FileInputStream(f);
        } catch (Exception ex) {
            System.out.println("Invalid file name");
            System.exit(1);
        }
        try (BufferedInputStream bufferedInputStream = new BufferedInputStream(fis)) {
            long len = getFileLen(file);
            writeFirmware(bufferedInputStream, len);
        } catch (IOException ex) {
            Logger.getLogger(ST_decrypt.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    private static int pack_u32(final int n, final int n2, final int n3, final int n4) {
        return (n << 24 & 0xFF000000) + (n2 << 16 & 0xFF0000) + (n3 << 8 & 0xFF00) + (n4 & 0xFF);
    }

    private static int u32(final int n) {
        return n >>> 24;
    }

    private static int u16(final int n) {
        return n >> 16 & 0xFF;
    }

    private static int u8(final int n) {
        return n >> 8 & 0xFF;
    }

    /**
     * Converts the key from String to byte array
     *
     * @param s input string
     * @param array destination array
     */
    public static void str_to_arr(final String s, final byte[] array) {
        final char[] charArray = s.toCharArray();
        for (int i = 0; i < 16; ++i) {
            array[i] = (byte) charArray[i];
        }
    }

    private static void key_decode(final byte[] array, final int[] array2) { // core.a.a(byte[], byte[])
        final int[] array3 = new int[4];
        for (int i = 0; i < 4; ++i) {
            array2[i] = (array3[i] = ByteBuffer.wrap(array).order(ByteOrder.LITTLE_ENDIAN).getInt(4 * i));
        }

        for (int j = 0; j < 10;) {
            array3[0] ^= (int) (pack_u32(aes_x[u16(array3[3])], aes_x[u8(array3[3])], aes_x[array3[3] & 0xFF], aes_x[u32(array3[3])]) ^ rcon[j++]);
            array3[1] ^= array3[0];
            array3[2] ^= array3[1];
            array3[3] ^= array3[2];

            System.arraycopy(array3, 0, array2, 4 * j, 4);
        }
    }

    /**
     * Encrypt firmware file
     */
    static void encrypt(final byte[] src, final byte[] dest, byte[] key, int len) {
        final byte[] array3 = new byte[16];
        int n = 0;
        key_decode(key, mystery_key);
        while (len > 0) {
            key = null;
            int n2;
            if (len >= 16) {
                key = Arrays.copyOfRange(src, n, n + 16);
                n2 = 16;
            } else if ((n2 = len) > 0) {
                final byte[] array4 = new byte[16];
                for (int j = 0; j < len; ++j) {
                    array4[j] = src[n + j];
                }
                for (int k = len; k < 16; ++k) {
                    array4[k] = (byte) Double.doubleToLongBits(Math.random());
                }
                key = array4;
            }
            if (len > 0) {
                final int[] a = mystery_key;
                final int[] tmp = new int[4];
                int n3 = 10;
                int n4 = 0;
                for (int l = 0; l < 4; ++l) {
                    tmp[l] = (ByteBuffer.wrap(key).order(ByteOrder.LITTLE_ENDIAN).getInt(4 * l) ^ a[l + 0]);
                }
                n4 += 4;
                do {
                    final int a2 = pack_u32(aes_x[u32(tmp[0])], aes_x[u16(tmp[1])], aes_x[u8(tmp[2])], aes_x[tmp[3] & 0xFF]);
                    final int a3 = pack_u32(aes_x[u32(tmp[1])], aes_x[u16(tmp[2])], aes_x[u8(tmp[3])], aes_x[tmp[0] & 0xFF]);
                    final int a4 = pack_u32(aes_x[u32(tmp[2])], aes_x[u16(tmp[3])], aes_x[u8(tmp[0])], aes_x[tmp[1] & 0xFF]);
                    final int a5 = pack_u32(aes_x[u32(tmp[3])], aes_x[u16(tmp[0])], aes_x[u8(tmp[1])], aes_x[tmp[2] & 0xFF]);
                    tmp[0] = (Crypto.polynom(a2) ^ a[n4]);
                    tmp[1] = (Crypto.polynom(a3) ^ a[n4 + 1]);
                    tmp[2] = (Crypto.polynom(a4) ^ a[n4 + 2]);
                    tmp[3] = (Crypto.polynom(a5) ^ a[n4 + 3]);
                    n4 += 4;
                } while (--n3 != 1);
                final int a6 = pack_u32(aes_x[u32(tmp[0])], aes_x[u16(tmp[1])], aes_x[u8(tmp[2])], aes_x[tmp[3] & 0xFF]);
                final int a7 = pack_u32(aes_x[u32(tmp[1])], aes_x[u16(tmp[2])], aes_x[u8(tmp[3])], aes_x[tmp[0] & 0xFF]);
                final int a8 = pack_u32(aes_x[u32(tmp[2])], aes_x[u16(tmp[3])], aes_x[u8(tmp[0])], aes_x[tmp[1] & 0xFF]);
                final int a9 = pack_u32(aes_x[u32(tmp[3])], aes_x[u16(tmp[0])], aes_x[u8(tmp[1])], aes_x[tmp[2] & 0xFF]);
                final int n5 = a6 ^ a[n4];
                final int n6 = a7 ^ a[n4 + 1];
                final int n7 = a8 ^ a[n4 + 2];
                final int n8 = a9 ^ a[n4 + 3];
                ByteBuffer.wrap(array3).order(ByteOrder.LITTLE_ENDIAN).putInt(n5);
                ByteBuffer.wrap(array3).order(ByteOrder.LITTLE_ENDIAN).putInt(4, n6);
                ByteBuffer.wrap(array3).order(ByteOrder.LITTLE_ENDIAN).putInt(8, n7);
                ByteBuffer.wrap(array3).order(ByteOrder.LITTLE_ENDIAN).putInt(12, n8);
            }
            for (int i = 0; i < 16; ++i) {
                dest[n + i] = array3[i];
            }
            len -= n2;
            n += n2;
        }
    }

    /**
     * Decrypt firmware file
     *
     * @param src firmware file
     * @param dest destination array
     * @param key key
     * @param len array.length
     */
    static void decrypt(final byte[] src, final byte[] dest, byte[] key, int len) {
        final byte[] array3 = new byte[16];
        int n = 0;
        key_decode(key, mystery_key);
        while (len > 0) {
            key = null;
            int n2;
            if (len >= 16) {
                key = Arrays.copyOfRange(src, n, n + 16);
                n2 = 16;
            } else if ((n2 = len) > 0) {
                final byte[] array4 = new byte[16];
                for (int j = 0; j < len; ++j) {
                    array4[j] = src[n + j];
                }
                for (int k = len; k < 16; ++k) {
                    array4[k] = (byte) Double.doubleToLongBits(Math.random());
                }
                key = array4;
            }
            if (len > 0) {
                final int[] a = mystery_key;
                final int[] tmp = new int[4];
                int n3 = 10;
                int n4 = 40;
                for (int l = 0; l < 4; ++l) {
                    tmp[l] = (ByteBuffer.wrap(key).order(ByteOrder.LITTLE_ENDIAN).getInt(4 * l) ^ a[l + 40]);
                }
                n4 -= 8;
                do {
                    final int n5 = pack_u32(aes_y[u32(tmp[0])], aes_y[u16(tmp[3])], aes_y[u8(tmp[2])], aes_y[tmp[1] & 0xFF]) ^ a[n4 + 4];
                    final int n6 = pack_u32(aes_y[u32(tmp[1])], aes_y[u16(tmp[0])], aes_y[u8(tmp[3])], aes_y[tmp[2] & 0xFF]) ^ a[n4 + 5];
                    final int n7 = pack_u32(aes_y[u32(tmp[2])], aes_y[u16(tmp[1])], aes_y[u8(tmp[0])], aes_y[tmp[3] & 0xFF]) ^ a[n4 + 6];
                    final int n8 = pack_u32(aes_y[u32(tmp[3])], aes_y[u16(tmp[2])], aes_y[u8(tmp[1])], aes_y[tmp[0] & 0xFF]) ^ a[n4 + 7];
                    tmp[0] = Crypto.polynom2(n5);
                    tmp[1] = Crypto.polynom2(n6);
                    tmp[2] = Crypto.polynom2(n7);
                    tmp[3] = Crypto.polynom2(n8);
                    n4 -= 4;
                } while (--n3 != 1);
                final int n9 = pack_u32(aes_y[u32(tmp[0])], aes_y[u16(tmp[3])], aes_y[u8(tmp[2])], aes_y[tmp[1] & 0xFF]) ^ a[n4 + 4];
                final int n10 = pack_u32(aes_y[u32(tmp[1])], aes_y[u16(tmp[0])], aes_y[u8(tmp[3])], aes_y[tmp[2] & 0xFF]) ^ a[n4 + 5];
                final int n11 = pack_u32(aes_y[u32(tmp[2])], aes_y[u16(tmp[1])], aes_y[u8(tmp[0])], aes_y[tmp[3] & 0xFF]) ^ a[n4 + 6];
                final int n12 = pack_u32(aes_y[u32(tmp[3])], aes_y[u16(tmp[2])], aes_y[u8(tmp[1])], aes_y[tmp[0] & 0xFF]) ^ a[n4 + 7];
                ByteBuffer.wrap(array3).order(ByteOrder.LITTLE_ENDIAN).putInt(n9);
                ByteBuffer.wrap(array3).order(ByteOrder.LITTLE_ENDIAN).putInt(4, n10);
                ByteBuffer.wrap(array3).order(ByteOrder.LITTLE_ENDIAN).putInt(8, n11);
                ByteBuffer.wrap(array3).order(ByteOrder.LITTLE_ENDIAN).putInt(12, n12);
            }
            for (int i = 0; i < n2; ++i) {
                dest[n + i] = array3[i];
            }
            len -= n2;
            n += n2;
        }
    }

    static int[] mystery_key = new int[44];

    static int[] aes_x = {
        0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5, 0x30, 0x01, 0x67, 0x2B,
        0xFE, 0xD7, 0xAB, 0x76, 0xCA, 0x82, 0xC9, 0x7D, 0xFA, 0x59, 0x47, 0xF0,
        0xAD, 0xD4, 0xA2, 0xAF, 0x9C, 0xA4, 0x72, 0xC0, 0xB7, 0xFD, 0x93, 0x26,
        0x36, 0x3F, 0xF7, 0xCC, 0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
        0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A, 0x07, 0x12, 0x80, 0xE2,
        0xEB, 0x27, 0xB2, 0x75, 0x09, 0x83, 0x2C, 0x1A, 0x1B, 0x6E, 0x5A, 0xA0,
        0x52, 0x3B, 0xD6, 0xB3, 0x29, 0xE3, 0x2F, 0x84, 0x53, 0xD1, 0x00, 0xED,
        0x20, 0xFC, 0xB1, 0x5B, 0x6A, 0xCB, 0xBE, 0x39, 0x4A, 0x4C, 0x58, 0xCF,
        0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85, 0x45, 0xF9, 0x02, 0x7F,
        0x50, 0x3C, 0x9F, 0xA8, 0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5,
        0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2, 0xCD, 0x0C, 0x13, 0xEC,
        0x5F, 0x97, 0x44, 0x17, 0xC4, 0xA7, 0x7E, 0x3D, 0x64, 0x5D, 0x19, 0x73,
        0x60, 0x81, 0x4F, 0xDC, 0x22, 0x2A, 0x90, 0x88, 0x46, 0xEE, 0xB8, 0x14,
        0xDE, 0x5E, 0x0B, 0xDB, 0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C,
        0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79, 0xE7, 0xC8, 0x37, 0x6D,
        0x8D, 0xD5, 0x4E, 0xA9, 0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
        0xBA, 0x78, 0x25, 0x2E, 0x1C, 0xA6, 0xB4, 0xC6, 0xE8, 0xDD, 0x74, 0x1F,
        0x4B, 0xBD, 0x8B, 0x8A, 0x70, 0x3E, 0xB5, 0x66, 0x48, 0x03, 0xF6, 0x0E,
        0x61, 0x35, 0x57, 0xB9, 0x86, 0xC1, 0x1D, 0x9E, 0xE1, 0xF8, 0x98, 0x11,
        0x69, 0xD9, 0x8E, 0x94, 0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
        0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68, 0x41, 0x99, 0x2D, 0x0F,
        0xB0, 0x54, 0xBB, 0x16
    };

    static int[] aes_y = {
        0x52, 0x09, 0x6A, 0xD5, 0x30, 0x36, 0xA5, 0x38, 0xBF, 0x40, 0xA3, 0x9E,
        0x81, 0xF3, 0xD7, 0xFB, 0x7C, 0xE3, 0x39, 0x82, 0x9B, 0x2F, 0xFF, 0x87,
        0x34, 0x8E, 0x43, 0x44, 0xC4, 0xDE, 0xE9, 0xCB, 0x54, 0x7B, 0x94, 0x32,
        0xA6, 0xC2, 0x23, 0x3D, 0xEE, 0x4C, 0x95, 0x0B, 0x42, 0xFA, 0xC3, 0x4E,
        0x08, 0x2E, 0xA1, 0x66, 0x28, 0xD9, 0x24, 0xB2, 0x76, 0x5B, 0xA2, 0x49,
        0x6D, 0x8B, 0xD1, 0x25, 0x72, 0xF8, 0xF6, 0x64, 0x86, 0x68, 0x98, 0x16,
        0xD4, 0xA4, 0x5C, 0xCC, 0x5D, 0x65, 0xB6, 0x92, 0x6C, 0x70, 0x48, 0x50,
        0xFD, 0xED, 0xB9, 0xDA, 0x5E, 0x15, 0x46, 0x57, 0xA7, 0x8D, 0x9D, 0x84,
        0x90, 0xD8, 0xAB, 0x00, 0x8C, 0xBC, 0xD3, 0x0A, 0xF7, 0xE4, 0x58, 0x05,
        0xB8, 0xB3, 0x45, 0x06, 0xD0, 0x2C, 0x1E, 0x8F, 0xCA, 0x3F, 0x0F, 0x02,
        0xC1, 0xAF, 0xBD, 0x03, 0x01, 0x13, 0x8A, 0x6B, 0x3A, 0x91, 0x11, 0x41,
        0x4F, 0x67, 0xDC, 0xEA, 0x97, 0xF2, 0xCF, 0xCE, 0xF0, 0xB4, 0xE6, 0x73,
        0x96, 0xAC, 0x74, 0x22, 0xE7, 0xAD, 0x35, 0x85, 0xE2, 0xF9, 0x37, 0xE8,
        0x1C, 0x75, 0xDF, 0x6E, 0x47, 0xF1, 0x1A, 0x71, 0x1D, 0x29, 0xC5, 0x89,
        0x6F, 0xB7, 0x62, 0x0E, 0xAA, 0x18, 0xBE, 0x1B, 0xFC, 0x56, 0x3E, 0x4B,
        0xC6, 0xD2, 0x79, 0x20, 0x9A, 0xDB, 0xC0, 0xFE, 0x78, 0xCD, 0x5A, 0xF4,
        0x1F, 0xDD, 0xA8, 0x33, 0x88, 0x07, 0xC7, 0x31, 0xB1, 0x12, 0x10, 0x59,
        0x27, 0x80, 0xEC, 0x5F, 0x60, 0x51, 0x7F, 0xA9, 0x19, 0xB5, 0x4A, 0x0D,
        0x2D, 0xE5, 0x7A, 0x9F, 0x93, 0xC9, 0x9C, 0xEF, 0xA0, 0xE0, 0x3B, 0x4D,
        0xAE, 0x2A, 0xF5, 0xB0, 0xC8, 0xEB, 0xBB, 0x3C, 0x83, 0x53, 0x99, 0x61,
        0x17, 0x2B, 0x04, 0x7E, 0xBA, 0x77, 0xD6, 0x26, 0xE1, 0x69, 0x14, 0x63,
        0x55, 0x21, 0x0C, 0x7D
    };

    static long[] rcon = {
        0x01000000L, 0x02000000L, 0x04000000L, 0x08000000L, 0x10000000L,
        0x20000000L, 0x40000000L, 0xFFFFFFFF80000000L, 0x1B000000L, 0x36000000L
    };
}
