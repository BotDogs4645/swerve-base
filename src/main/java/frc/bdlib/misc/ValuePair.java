package frc.bdlib.misc;

public class ValuePair<K, V> {
    public static <K, V> ValuePair<K, V> of(K obj1, V obj2) {
        return new ValuePair<K,V>(obj1, obj2);
    }

    private K v1;
    private V v2;

    public ValuePair(K v1, V v2) {
        this.v1 = v1;
        this.v2 = v2;
    }

    public K getLeftValue() {
        return v1;
    }

    public V getRightValue() {
        return v2;
    }
}
